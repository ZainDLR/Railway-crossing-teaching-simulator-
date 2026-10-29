# Model, architecture and limitations

## Physical assumptions

One train travels in the positive x direction. Its front starts at -300 m at
20 m/s; its length is 40 m. The crossing spans [-5, +5] m. It is occupied when
the train front is at least -5 m and its rear is at most +5 m. An approach detector
is active from front x = -220 m until the rear clears +5 m. The two nominal
approach sensors measure the same predicate; they are duplicated, not diverse.

The train responds directly to `permit_train`: true accelerates at 2 m/s² up to
20 m/s; false brakes at 2 m/s² down to zero. This is an idealized, reliable train
protection interface, NOT a claim that a real crossing controller can directly
brake an approaching train. Communication delay, brake failure, adhesion and
human response are omitted. At 20 m/s, the ideal stopping distance is 100 m,
plus distance traveled before the braking request. The approach location is
chosen to allow this model to stop well before the crossing in supported cases.

The barrier moves linearly from 0 (open) to 1 (closed) in four seconds. A command
can reverse its direction. A stuck barrier remains at its current position.
`barrier_closed` is true at fraction >= 0.999. The obstacle scenario represents
an object in the gate/crossing clearance envelope from t=0 up to but excluding
15 s; it then disappears. There is no general traffic or collision dynamics.
The train continues moving after a hazard, so the trace records consequences
without terminating the experiment.

## Timing and thread ownership

The main thread owns the physical model, fault injection and monitor. Every
20 ms of virtual time it records the current state and then advances the plant.
Every fifth tick (100 ms), it samples sensors and requests a controller step.
The worker thread owns the Controller object and its persistent fault latch.
A mutex protects the mailbox. A condition variable signals requests and
responses; waits always check a predicate in a loop to tolerate spurious wakes.
The main thread waits for the response before proceeding to the next plant tick.

Tick ordering:
1. Set scheduled faults/obstacle state for the current virtual time.
2. On controller ticks, sample sensors unless frozen and obtain a new command
   unless command updates are suppressed.
3. Check command age and choose the effective actuator command.
4. Monitor current physical truth and log the state/commands.
5. Advance physics by 20 ms (except at the last sample).

The simulation ends at the largest 20 ms tick not exceeding `--duration`.
At a closing threshold, the first sensor observation is on a controller tick.
All logged timestamps and watchdog decisions use virtual time. Wall-clock
pacing uses `CLOCK_MONOTONIC` and absolute `clock_nanosleep` deadlines so a
relative sleep does not accumulate a new delay each step. If the host is late,
the loop catches up; it does not skip physics steps or simulate a late controller.

The plant uses trapezoidal position integration and bounded speed/barrier
updates. Hazard checks are sampled, not continuous-time proofs. CSV rounding is
for presentation only; controller and plant calculations use double precision.

## Fault definitions

Unless specified otherwise, faults activate at t=2 s and persist:

| Fault | Injection |
|---|---|
| sensor-stale | Entire sensor packet, including timestamp, freezes at t=1.9 s. |
| sensor-a-stuck | Approach A reports false with a fresh timestamp; B remains correct. |
| common-cause | Both approach sensors report false; occupancy and other sensors remain correct. |
| barrier-stuck | Mechanical position stops changing despite actuator commands. |
| controller-freeze | No new controller calls/commands after t=1.9 s. |
| road-obstacle | Physical obstacle and its sensor are true on [0,15) s. |

The watchdog fires when virtual command age is strictly greater than 0.5 s.
It requests braking and preserves the last gate target. It does not decide that
blindly closing a gate is always safe. Reference freshness checks also use
strictly greater than 0.5 s, but execute only on 100 ms controller ticks.
These thresholds are exercise choices, not railway engineering requirements.

## Monitor and outputs

The monitor does not intervene. It reads ground truth independently of sensors:

- H1: train overlaps crossing while the barrier is not fully closed.
- H2: train overlaps crossing while an obstacle is present.
- H3: obstacle is present while barrier fraction is above 0.001.

H1 is a protective-condition violation, not necessarily an actual collision.
H3 deliberately represents a conservative gate-clearance conflict, not a force
or impact model. A hazard episode begins when the OR of H1/H2/H3 changes from
false to true. `hazard_samples` counts ticks with any hazard, not independent
failures. Neither quantity is a failure probability or reliability estimate.
`train_cleared` records whether the rear has passed +5 m at the end.

CSV distinguishes requested `command_*` from watchdog-adjusted `effective_*`.
`sampled_at_s` exposes stale packets. The monitor's `ground_occupied` remains
truthful even when readings are corrupted. CSV can be opened in a spreadsheet
or plotted with a tool of your choice. Compare barrier fraction, train position,
commands and hazard flags against time.

## Extensions for later tutorials

This implementation is intentionally a small starting point. To teach actual
scheduling and priority inversion, add a separate asynchronous execution mode:

- Run plant and controller as independent periodic POSIX threads.
- Timestamp readings and commands with the monotonic wall clock.
- Advance physics by an explicitly chosen policy when the host is late.
- Measure release lateness, execution time and response time separately.
- Add bounded workload and a command watchdog that can run while the controller
  is blocked. Do not keep the present lockstep wait for that mode.
- Introduce controlled resource contention and compare mutex protocols on a
  configured Linux teaching machine. Scheduling policies and privileges must be
  checked on that machine; a VM's timing cannot establish target guarantees.

Keep deterministic mode for reproducible fault tests. Other useful model
extensions include delayed packets, faulty feedback, sensor plausibility tests,
a reset protocol, multiple trains, configurable train braking and independent
train detection. Do not claim common-cause tolerance simply by adding identical
sensors. A GUI can later consume the trace or receive state snapshots without
changing the student interface.
