#include "api.hpp"
#include <pthread.h>
#include <time.h>
#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <set>

namespace {
constexpr double dt = 0.02;
void check(int code) {
    if (code) { std::cerr << "POSIX error: " << std::strerror(code) << '\n'; std::exit(1); }
}
struct Options {
    std::string scenario = "nominal", csv = "trace.csv";
    bool reference = true, realtime = false, quiet = false;
    double duration = 45;
};
const std::set<std::string> scenarios = {
    "nominal", "sensor-stale", "sensor-a-stuck", "common-cause",
    "barrier-stuck", "controller-freeze", "road-obstacle"
};
Options parse(int argc, char** argv) {
    Options o;
    for (int i=1; i<argc; ++i) {
        std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) throw std::runtime_error("Missing value for " + arg);
            return argv[i];
        };
        if (arg == "--scenario") o.scenario = value();
        else if (arg == "--controller") {
            auto v=value();
            if (v != "reference" && v != "baseline") throw std::runtime_error("Unknown controller");
            o.reference = v == "reference";
        } else if (arg == "--csv") o.csv=value();
        else if (arg == "--duration") {
            auto v=value(); size_t used=0; o.duration=std::stod(v, &used);
            if (used != v.size() || !std::isfinite(o.duration) || o.duration <= 0 || o.duration > 3600)
                throw std::runtime_error("Duration must be in (0, 3600]");
        } else if (arg == "--realtime") o.realtime=true;
        else if (arg == "--quiet") o.quiet=true;
        else if (arg == "--help") {
            std::cout << "Usage: ./crossing [--controller reference|baseline]\n"
                      << "  [--scenario nominal|sensor-stale|sensor-a-stuck|common-cause|barrier-stuck|controller-freeze|road-obstacle]\n"
                      << "  [--duration SECONDS] [--csv PATH] [--realtime] [--quiet]\n"
                      << "Exit: 0 no sampled hazard; 2 hazard observed; 1 input/I/O error.\n";
            std::exit(0);
        } else throw std::runtime_error("Unknown argument: " + arg);
    }
    if (!scenarios.count(o.scenario)) throw std::runtime_error("Unknown scenario: " + o.scenario);
    return o;
}
// One request and response at a time. All mailbox fields are protected by mu.
// This deliberate lockstep makes the physics and fault traces repeatable.
struct Mailbox {
    pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t changed = PTHREAD_COND_INITIALIZER;
    bool pending=false, ready=false, stop=false, reference=true;
    double now=0;
    Sensors sensors;
    Command command;
};
void* worker(void* arg) {
    auto& m=*static_cast<Mailbox*>(arg);
    Controller controller;
    check(pthread_mutex_lock(&m.mu));
    while (true) {
        while (!m.pending && !m.stop) check(pthread_cond_wait(&m.changed, &m.mu));
        if (m.stop) break;
        Sensors s=m.sensors; double now=m.now; bool ref=m.reference;
        check(pthread_mutex_unlock(&m.mu));
        Command c=controller.step(s, now, ref);
        check(pthread_mutex_lock(&m.mu));
        m.command=c; m.pending=false; m.ready=true;
        check(pthread_cond_broadcast(&m.changed));
    }
    check(pthread_mutex_unlock(&m.mu));
    return nullptr;
}
class ControllerThread {
    Mailbox m;
    pthread_t thread{};
public:
    explicit ControllerThread(bool reference) {
        m.reference=reference; check(pthread_create(&thread, nullptr, worker, &m));
    }
    ~ControllerThread() {
        check(pthread_mutex_lock(&m.mu)); m.stop=true;
        check(pthread_cond_broadcast(&m.changed)); check(pthread_mutex_unlock(&m.mu));
        check(pthread_join(thread, nullptr));
        check(pthread_cond_destroy(&m.changed)); check(pthread_mutex_destroy(&m.mu));
    }
    Command step(const Sensors& s, double now) {
        check(pthread_mutex_lock(&m.mu));
        m.sensors=s; m.now=now; m.pending=true; m.ready=false;
        check(pthread_cond_broadcast(&m.changed));
        while (!m.ready) check(pthread_cond_wait(&m.changed, &m.mu));
        Command c=m.command; check(pthread_mutex_unlock(&m.mu)); return c;
    }
};
struct Plant {
    double x=-300, speed=20, barrier=0; // train front in metres; barrier 0=open, 1=closed
    bool occupied() const { return x >= -5 && x-40 <= 5; }
    bool approaching() const { return x >= -220 && x-40 <= 5; }
    void advance(Command c, bool stuck) {
        const double old_speed=speed;
        speed=std::clamp(speed + (c.permit_train ? 2.0 : -2.0)*dt, 0.0, 20.0);
        x += (old_speed+speed)*0.5*dt;
        if (!stuck) barrier=std::clamp(barrier+(c.close_barrier ? 1.0 : -1.0)*dt/4.0, 0.0, 1.0);
    }
};
void pace(const timespec& start, double elapsed) {
    timespec deadline=start;
    const long long ns=static_cast<long long>(std::llround(elapsed*1e9));
    deadline.tv_sec += ns/1000000000;
    deadline.tv_nsec += ns%1000000000;
    if (deadline.tv_nsec >= 1000000000) { ++deadline.tv_sec; deadline.tv_nsec-=1000000000; }
    int rc;
    do { rc=clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr); } while (rc==EINTR);
    check(rc);
}
}
int main(int argc, char** argv) {
    try {
        Options o=parse(argc, argv);
        std::ofstream csv(o.csv);
        if (!csv) throw std::runtime_error("Cannot open CSV: " + o.csv);
        csv << "time_s,train_front_m,train_speed_mps,barrier_fraction,ground_occupied,road_obstacle,approach_a,approach_b,sampled_at_s,command_close,command_permit,effective_close,effective_permit,watchdog,hazard_unprotected,hazard_obstacle,hazard_gate\n";
        csv << std::fixed << std::setprecision(3);
        ControllerThread controller(o.reference);
        Plant p;
        Sensors s;
        Command command;
        double last_command=0;
        int hazard_samples=0, episodes=0;
        bool previous_hazard=false, saw_watchdog=false;
        timespec start{};
        if (clock_gettime(CLOCK_MONOTONIC, &start)) throw std::runtime_error("clock_gettime failed");
        const int ticks=static_cast<int>(std::floor(o.duration/dt));
        for (int tick=0; tick<=ticks; ++tick) {
            double now=tick*dt;
            if (o.realtime) pace(start, now);
            bool active=now>=2.0;
            bool obstacle=o.scenario=="road-obstacle" && now<15.0;
            // Sensors sample every controller period. Staleness preserves timestamp AND values.
            if (tick%5==0) {
                if (!(active && o.scenario=="sensor-stale")) {
                    s={now, p.approaching(), p.approaching(), p.occupied(), p.barrier>=0.999, obstacle};
                    if (active && (o.scenario=="sensor-a-stuck" || o.scenario=="common-cause")) s.approach_a=false;
                    if (active && o.scenario=="common-cause") s.approach_b=false;
                }
                // Inject missing updates, not an actual hanging worker (which would block lockstep).
                if (!(active && o.scenario=="controller-freeze")) {
                    command=controller.step(s, now); last_command=now;
                }
            }
            // Independent actuator-side command-age watchdog, shared by both controllers.
            bool watchdog=now-last_command>0.5;
            Command effective=command;
            if (watchdog) effective={command.close_barrier, false}; // hold gate target, request brake
            if (watchdog && !saw_watchdog && !o.quiet)
                std::cout << "t=" << now << " WATCHDOG: brake requested\n";
            saw_watchdog |= watchdog;
            // Independent sampled monitor reads physical truth, never corrupted sensors.
            bool h1=p.occupied() && p.barrier<0.999;
            bool h2=p.occupied() && obstacle;
            bool h3=obstacle && p.barrier>0.001;
            bool hazard=h1 || h2 || h3;
            if (hazard) ++hazard_samples;
            if (hazard && !previous_hazard) {
                ++episodes;
                if (!o.quiet) std::cout << "t=" << now << " HAZARD unprotected=" << h1
                    << " train/obstacle=" << h2 << " gate/obstacle=" << h3 << '\n';
            }
            previous_hazard=hazard;
            csv << now << ',' << p.x << ',' << p.speed << ',' << p.barrier << ',' << p.occupied()
                << ',' << obstacle << ',' << s.approach_a << ',' << s.approach_b << ',' << s.sampled_at
                << ',' << command.close_barrier << ',' << command.permit_train
                << ',' << effective.close_barrier << ',' << effective.permit_train
                << ',' << watchdog << ',' << h1 << ',' << h2 << ',' << h3 << '\n';
            if (!o.quiet && tick%50==0)
                std::cout << std::fixed << std::setprecision(1) << "t=" << now << "s x=" << p.x
                    << "m v=" << p.speed << "m/s gate=" << p.barrier*100 << "%\n";
            if (tick<ticks) p.advance(effective, active && o.scenario=="barrier-stuck");
        }
        csv.flush();
        if (!csv) throw std::runtime_error("Writing CSV failed");
        std::cout << "SUMMARY scenario=" << o.scenario << " controller=" << (o.reference?"reference":"baseline")
            << " hazard_samples=" << hazard_samples << " hazard_episodes=" << episodes
            << " watchdog=" << saw_watchdog << " train_cleared=" << (p.x-40>5)
            << " final_speed=" << p.speed << "\n";
        return hazard_samples ? 2 : 0;
    } catch (const std::exception& e) { std::cerr << "Error: " << e.what() << '\n'; return 1; }
}
