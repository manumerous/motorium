# motorium_drake

A [Drake](https://drake.mit.edu) physics simulator behind the motorium HAL. `DrakeDriver` is a `hal::DriverBase`, so a controller that runs on `MujocoDriver` runs on Drake by changing one line:

```cpp
auto& sim = hal.addDriver<motorium::drake_sim::DrakeDriver>(config);  // instead of MujocoDriver
```

## Quick start

```cpp
#include <motorium_drake/DrakeDriver.h>
#include <motorium_hal/RobotHAL.h>

using namespace motorium;

hal::RobotHAL hal("robot.urdf");  // the joints of this model are what the controller sees

drake_sim::DrakeSimConfig config;  // defaults: dt = 0.5 ms, ground on, headless
config.headless = false;           // show the robot in Meshcat (URL printed on startup)
auto& sim = hal.addDriver<drake_sim::DrakeDriver>(config);  // loads robot.urdf into Drake

model::RobotState state(hal.getRobotDescription());
model::RobotJointFeedbackAction action(hal.getRobotDescription());

hal.startDrivers();  // starts the physics thread
while (hal.getState() == hal::DriverState::RUNNING) {
  hal.updateState(state);                                  // latest simulated state
  controller.computeJointControlAction(state.getTime(), state, desired, action);
  hal.setAction(action);                                   // applied from the next physics step
  // ... sleep until the next control period
}
```

## How it works

```
RobotHAL ──updateState()/setAction()──► DrakeDriver                    (public, include/)
                                         ├─ physics thread: step every dt
                                         ├─ state snapshot, last action, faults
                                         ├─ DrakeSimulation             (private, src/)
                                         │    Drake plant + conversion motorium ⇄ Drake
                                         └─ DrakeMeshcatVisualizer      (optional, own thread)
```

- **`DrakeSimulation`** builds a Drake `MultibodyPlant` from the model file and translates between motorium types (indexed by `RobotDescription` joint index) and Drake's state and actuation vectors. Name lookups happen once, at construction.
- **`DrakeDriver`** owns a `DrakeSimulation`, runs the physics thread, and implements the HAL interface.
- **`DrakeMeshcatVisualizer`** poses a visual-only copy of the robot from the driver's state.

### One physics step

1. Compute each actuated joint's torque from the latest action at the current state: `kp·(q_des − q) + kd·(v_des − v) + feed_forward_effort`. This matches `MujocoDriver`. The gains are evaluated every physics step, so implicit PD controllers run at the physics rate.
2. Clamp the torque to the joint's torque bounds from the `RobotDescription`; replace non-finite torques with 0. Drake does not clamp actuation itself.
3. Advance Drake by `dt`.
4. Copy the new state into the snapshot that `updateState()` reads.
5. Check for faults (see below).

Until the first `setAction()`, the driver applies `config.initialActionPtr` (or zero torque), so the robot can hold its pose while the controller starts.

### The simulated model

- Point contact with Drake's SAP solver.
- Only contacts with the ground are simulated; collisions between the robot's own bodies are disabled. Put collision geometry only where contact matters (e.g. small spheres under the feet).
- Every `RobotDescription` joint gets viscous damping `defaultJointDamping`, and an actuator if the model has none (URDFs without `<transmission>` tags). The actuator's effort limit is the joint's torque bound.
- A floating base is detected automatically; a URDF needs no floating joint.

### Threads and locking

| Thread | Does | Locks |
|---|---|---|
| physics (or the caller of `advance()`) | steps the simulation | `sim_mutex_` for the step; `action_mutex_` only while computing torques; `snapshot_mutex_` only while copying the new state |
| controller (`updateState()`, `setAction()`) | copies the snapshot / stores the action | `snapshot_mutex_` / `action_mutex_` for the copy only |
| Meshcat (optional) | reads the snapshot at `visualizationRateHz` | `snapshot_mutex_` for the copy only |

The controller never waits for a physics step (about 300 µs for the 29-DoF Unitree G1). The physics thread wakes every `dt` of real time. If it falls more than one step behind, it resynchronizes instead of catching up in a burst, and counts an overrun in `getMetrics()`.

### Lifecycle and faults

| HAL call | DrakeDriver |
|---|---|
| `addDriver()` | builds the simulation at the initial state; physics not running (`READY`) |
| `startDrivers()` | starts the physics thread (`RUNNING`) |
| `stopDrivers()` | stops and joins the physics thread (`READY`) |
| `resetDrivers()` | restores the initial state and the initial action; allowed while running |

A Drake error, a non-finite state, or the floating base dropping below `faultBelowRootHeight` puts the driver into `FAULT` and stops the physics. `RobotHAL` safety-stops all drivers on its next call. To recover, the application calls `resetDrivers()` and then `startDrivers()`; physics stays paused in between, for as long as the application needs (e.g. to reset its MPC).

## Configuration (`DrakeSimConfig`)

| Field | Default | Meaning |
|---|---|---|
| `modelPath` | the `RobotDescription`'s model | URDF or SDF to simulate |
| `packagePaths` | none | `{name, directory}` pairs to resolve `package://` URIs |
| `initStatePtr` | joints at 0, base at z = 1 m | state at construction and after reset |
| `initialActionPtr` | zero torque | action until the first `setAction()` |
| `dt` | 0.0005 s | physics time step |
| `defaultJointDamping` | 10 Nms/rad | viscous damping of every joint |
| `ground` | on; friction 2.0 / 1.5, dissipation 2.0, stiffness 5e4 N/m | flat ground (top face at z = 0) |
| `contactBodyNames` | none | bodies that set contact flag 0, 1, …; empty = flags not written |
| `faultBelowRootHeight` | off | fault when the floating base drops below this height |
| `manualStepping` | false | no physics thread; step with `advance(n)` |
| `headless` | true | false = start the Meshcat visualizer |
| `visualizationRateHz` | 30 | Meshcat update rate |
| `meshcatHost` / `meshcatPort` | `localhost` / first free from 7000 | Meshcat server address |
| `verbose` | false | print the loaded model's size, and every 5 s the real-time factor and step-time metrics |

The ground and damping defaults are the values the Unitree G1 walks with in wb_humanoid_mpc's Drake simulation. The damping is high for light robots: a 1 kg, 0.5 m pendulum is heavily overdamped.

## Conventions

The driver fills `RobotState` exactly like `MujocoDriver`:

| Field | Meaning |
|---|---|
| root position | floating base (or the body attached to the world), world frame |
| root orientation | rotation local → world |
| root linear / angular velocity | **local** (root) frame |
| joint position, velocity | per `RobotDescription` joint |
| joint effort | torque applied in the last step (after clamping) |
| time | simulation time; keeps running across resets |
| contact flag *i* | body `contactBodyNames[i]` touches something; only written when `contactBodyNames` is set |

## Building and testing

Drake comes from [`drake_vendor`](../../lib/drake_vendor), which downloads the prebuilt Drake release for Ubuntu 24.04 (x86_64 or arm64) and puts it on `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH` once `install/setup.bash` is sourced. Its system packages (Eigen, fmt, spdlog, BLAS, LAPACK) are listed in the vendor's README.

```bash
# from the motorium repository root (or from wb_humanoid_mpc, which includes motorium)
make build-motorium_drake
make test-motorium_drake
```

Tested with Drake v1.59.0 (`drake_vendor`) using clang 19 / C++23 with `-Werror=thread-safety-analysis` (motorium's CI flags) and GCC 13.3 / C++20. Don't also keep a local Drake install on `CMAKE_PREFIX_PATH` / `LD_LIBRARY_PATH`: it can shadow the vendored one.

The tests (`test/`) use two small primitive-geometry robots: a fixed-base pendulum (`pendulum_fixed_base.urdf`) and a floating-base four-legged table (`floating_base_quadruped.urdf`). Most tests use `manualStepping`, so they are deterministic and run faster than real time.

| Test | Covers |
|---|---|
| `test_drake_sim_config` | defaults, `validate()` |
| `test_drake_driver_lifecycle` | start / stop / restart, real-time pacing, reset, invalid models |
| `test_drake_driver_physics` | pendulum period vs. theory, PD hold, torque clamping, NaN actions, damping, initial action |
| `test_drake_driver_floating_base` | free fall, local-frame root velocities, standing with foot contacts, fall fault and recovery |
| `test_drake_meshcat_visualizer` | Meshcat serves a URL while the simulation runs |

## Caveats

- **Compiler mismatch with a prebuilt Drake.** Drake passes the alias template `Link = RigidBody` as a template argument, which GCC and clang mangle differently. Code compiled with clang that calls `MultibodyPlant::get_body(BodyIndex)` does not link against a GCC-built Drake (such as a default source build on Ubuntu; check how a prebuilt Drake was compiled). This package reaches bodies through joints instead (`src/DrakeModelLoading.h`); keep it that way when adding code.
- **Meshcat is single-threaded.** Drake requires every Meshcat call from the thread that created the instance; the visualizer therefore builds everything on its own thread.
- **Real-time budget.** A Drake step for the 29-DoF G1 takes about 300 µs (mean) of the 500 µs budget, against 50–80 µs for MuJoCo. Watch `getMetrics().overrunCount`; keep other heavy threads (e.g. an MPC solver) on other CPU cores.
