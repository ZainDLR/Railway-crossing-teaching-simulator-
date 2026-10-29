#pragma once
// Student-facing interface. No access to the physical model's ground truth.
struct Sensors {
    double sampled_at = 0;
    bool approach_a = false, approach_b = false;
    bool occupied = false, barrier_closed = false, road_obstacle = false;
};
struct Command {
    bool close_barrier = false;
    bool permit_train = false; // false requests braking; braking is NOT instantaneous
};
class Controller {
    bool fault_latched_ = false;
public:
    Command step(const Sensors& s, double now, bool reference);
};
