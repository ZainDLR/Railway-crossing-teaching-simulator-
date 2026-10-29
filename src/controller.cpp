#include "api.hpp"

Command Controller::step(const Sensors& s, double now, bool reference) {
    if (!reference) {
        // Deliberately weak baseline: trusts one sensor, ignores freshness,
        // actuator feedback and obstacles, and never requests train braking.
        return {s.approach_a || s.occupied, true};
    }
    // Example solution under the documented model assumptions, not a certified
    // railway controller. A fault requires an operator reset (restart this demo).
    if (now - s.sampled_at > 0.5 || s.approach_a != s.approach_b)
        fault_latched_ = true;
    const bool demand = s.approach_a || s.approach_b || s.occupied;
    Command c;
    c.close_barrier = (demand || fault_latched_) && !s.road_obstacle;
    c.permit_train = !fault_latched_ && !s.road_obstacle &&
                     (!demand || s.barrier_closed);
    return c;
}
