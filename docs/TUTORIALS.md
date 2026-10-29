# Seven tutorials using one evolving case study

This sequence is a proposed adaptation to the implemented first version.
It covers safety and POSIX synchronization; scheduling/priority inversion need
the asynchronous extension described in MODEL.md and are not claimed here.

## 1 — Hazards, assumptions and nominal operation

Build and run the nominal case with both controllers. Read the model assumptions
before inspecting code. Draw the system boundary and identify the train brake
interface, sensors, controller, barrier and monitor. State what the environment
must guarantee for the controller's safety argument to hold.

Deliverable: three hazards and corresponding testable requirements. Example:
"The train must not occupy the crossing unless barrier closure is confirmed."
Distinguish a close command from feedback that closure has completed. Estimate
100 m stopping distance and account for detection/response delay.

## 2 — Controller state and safe degradation

Implement explicit OPEN, CLOSING, CLOSED and FAULT states in the student module.
Inspect how a reference controller pauses the train while closing. Run the stuck
barrier scenario. Add a closure timeout with an explicit diagnostic state;
maintain braking rather than interpreting the timeout as successful closure.

Deliverable: transition table, assumptions and traces. A train stopped forever
can meet some safety requirements while failing an availability requirement.
Require nominal progress so "always stop" is not a sufficient solution.

## 3 — POSIX threads and coherent data transfer

Inspect the worker/main mailbox. Explain mutex ownership, predicate-based waits,
spurious wakeups, and why the command is copied under the mutex. Add a sensor
packet sequence number, transfer it with the packet, and echo it in a diagnostic
response to demonstrate coherent handoff.

Deliverable: annotated synchronization explanation. This lockstep exercise does
not demonstrate priority inversion. Do not remove a mutex and treat a C++ data
race as a deterministic model of a particular hardware fault: such races invoke
undefined behavior. Use an explicitly injected inconsistent packet if needed.

## 4 — Freshness, command age and watchdogs

Compare sensor-stale with controller-freeze. In the former, commands continue
but are based on old observations; in the latter, commands stop arriving.
Explain why the command watchdog alone cannot detect stale sensor content.
Vary the freshness threshold in the controller and compare stopping positions.

Deliverable: distinguish sensor timestamp, command timestamp, timeout and actual
braking distance. Thresholds in this model use simulation time, not measured OS
response times. Analyze boundary behavior at exactly 0.5 s.

## 5 — Redundancy and common-cause faults

Run sensor-a-stuck and common-cause. Implement a disagreement response and
explain why agreement is insufficient to establish correctness. The occupancy
sensor detects a train only once it reaches the crossing; that is too late for
an initially open barrier in the common-cause case.

Deliverable: a fault table and a proposed genuinely independent detection or
protection mechanism. State its assumptions before changing the simulator.
The existing common-cause test deliberately expects a hazard.

## 6 — Obstacles and conflicting safety actions

Run road-obstacle. Explain why closing the barrier regardless of circumstances
can create another hazard. Modify the obstacle interval in the instructor model
and consider obstacles appearing after closure; the initial reference is not a
solution to every such scenario.

Deliverable: justified responses for known obstacle, unknown sensor state and
actuator failure. Identify cases where the physical system can no longer avoid
all hazards, regardless of controller code.

## 7 — Verification and safety argument

Create a requirement-to-test table. Execute normal, individual-fault and
common-cause cases, recording both safety and successful train passage. Repeat
runs to compare deterministic traces. Add tests for controller state boundaries
and one additional fault. Explain the limits of the finite scenario set.

Deliverable: controller code, traces, test results and a short argument linking
requirements, design decisions, evidence and unresolved assumptions. Do not
report a passing test suite as proof of safety or a railway certification claim.

## Suggested assessment criteria

- Correct safety requirements and explicit physical assumptions.
- Nominal progress and defensible degraded behavior.
- Correct synchronization and coherent state transfer.
- Useful fault-detection logic and transparent limitations.
- Tests with independent physical-state checks.
- Clear distinction between safety, reliability and availability.
