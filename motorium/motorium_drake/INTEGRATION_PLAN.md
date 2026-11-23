# motorium_drake: Integration Plan

Status: **driver implemented, unit-tested and documented** (M1–M4 except the example; see section 15 and [README.md](README.md)). Not committed yet.

| Repo | Branch |
|---|---|
| motorium (fork `moavia90/motorium`) | `feature/moavia-drake-integration` |
| wb_humanoid_mpc | `feature/moavia-drake-motorium-integration` |

## 1. Goal

Add a [Drake](https://drake.mit.edu) physics backend to motorium as a `hal::DriverBase` implementation. Controllers then run unchanged on MuJoCo or Drake by swapping one `addDriver<...>()` call:

```cpp
motorium::hal::RobotHAL hal(urdf_path);
auto& sim = hal.addDriver<motorium::drake::DrakeDriver>(drake_config);   // instead of MujocoDriver
```

The behaviour to reproduce is the Drake simulation that already walks the Unitree G1 in `wb_humanoid_mpc` (branch `moavia-drake-integration-legacy`, package `robot_runtime/humanoid_drake_sim`). That code targets the older `robot_model::RobotHWInterfaceBase` API and mixes physics with OCS2, ROS and G1-specific control logic.

### In scope (this package)

- Drake `MultibodyPlant` + `SceneGraph` simulation of a URDF/SDF model, stepped on a dedicated real-time-paced thread.
- Joint feedback actions applied as `kp·(q_des − q) + kd·(v_des − v) + feed_forward_effort` each physics step, clamped to torque limits.
- Floating- and fixed-base robots, ground plane, optional contact flags.
- Optional Meshcat visualization (the counterpart of `MujocoRenderer`).
- Faults reported through the `DriverBase` lifecycle.
- Unit tests, an example, colcon and Bazel build files.

### Out of scope (stays in the consuming application)

These parts of the legacy `drake_mrt_sim_node` are application logic and must **not** go into motorium:

| Legacy feature | Belongs in |
|---|---|
| OCS2 MRT interface, Pinocchio inverse dynamics, finite-difference `qdd_des` | `wb_humanoid_mpc` controller |
| Per-joint tracking gains for G1 | `wb_humanoid_mpc` config |
| Velocity-command clamping / soft saturation, stop alignment | `wb_humanoid_mpc` |
| Startup warm-up (`/mpc_reset`, policy knot-height check) | `wb_humanoid_mpc` |
| Fall *recovery* choreography (MPC reset, `mpcTimeOffset`) | `wb_humanoid_mpc` |
| Spawn-height grounding offset | `wb_humanoid_mpc`, passed in via `initStatePtr` |
| ROS nodes, topics, telemetry logs, CPU pinning with `taskset` | `wb_humanoid_mpc` / launch files |

Section 9 covers the consumer-side work needed to reach walking parity.

## 2. Reference material

| What | Where |
|---|---|
| Legacy physics backend (main port source) | legacy `robot_runtime/humanoid_drake_sim/src/DrakeSimInterface.cpp` + `include/humanoid_drake_sim/DrakeSimInterface.h` |
| Legacy walking node + launch | `.../src/drake_mrt_sim_node.cpp`, `robot_models/unitree_g1/g1_centroidal_mpc/launch/drake_mrt_sim.launch.py` |
| Legacy design notes and failure analysis | `.../humanoid_drake_sim/Documentation.md` |
| Drake-ready G1 model | legacy `robot_models/unitree_g1/g1_description/urdf/g1_29dof_drake.urdf` |
| Existing motorium sim driver (pattern to follow) | `motorium/motorium_mujoco/` |
| Driver contract | `motorium/motorium_hal/include/motorium_hal/DriverBase.h` |

Local checkout of the legacy branch: `~/Documents/humanoid_mpc_legacy_ws/src/wb_humanoid_mpc` (git worktree).

Drake used during development: source build `v1.52.0-94-g38121ab8`, installed at `~/drake-build/install`.

## 3. Motorium rules this package must follow

1. **Name → index resolution only at construction.** No string lookups, `GetJointByName()`, `GetBodyByName()` or heap allocation in the physics loop, `updateStateImpl()` or `setActionImpl()`.
2. **Only `RobotHAL` creates the driver.** Constructor takes `hal::HalKey` first.
3. **`DriverBase` owns the lifecycle.** Implement only the `*Impl()` hooks. Call `markReady()` at the end of the constructor and `reportFault()` on errors. Never change state any other way.
4. **No silent recovery.** Faults (fall, NaN state, Drake exception) are reported. The application decides whether to call `resetDrivers()`.
5. **Compiler-checked thread safety.** `absl::Mutex` + `ABSL_GUARDED_BY`; must build under `-Wthread-safety -Werror=thread-safety-analysis`. Threads are `std::jthread` with `std::stop_token`.
6. **Configuration is a struct with `validate()`**, failing fast with `MT_CHECK`.
7. **Robot-agnostic, ROS-free source.** No `ament_index_cpp`, no G1 names as defaults. ament only in build glue.
8. **Both build systems** (colcon + Bazel), clang-format (Google, 140 columns, `make format`), BSD-3 license header, gtest tests.

## 4. Package layout

```
motorium/motorium_drake/
├── INTEGRATION_PLAN.md              # this file (replace with README.md once implemented)
├── CMakeLists.txt
├── package.xml
├── BUILD.bazel
├── include/motorium_drake/
│   ├── DrakeSimConfig.h             # config struct + validate()
│   ├── DrakeDriver.h                # hal::DriverBase implementation
│   ├── DrakeMeshcatVisualizer.h     # optional visualization thread
│   └── DrakeMetrics.h               # RTF / overrun / step-time metrics
├── src/
│   ├── DrakeSimConfig.cpp
│   ├── DrakeDriver.cpp
│   └── DrakeMeshcatVisualizer.cpp
└── test/
    ├── resources/
    │   ├── pendulum_fixed_base.urdf # 1-DoF, primitive geometry, no meshes
    │   └── biped_floating_base.urdf # small floating-base biped, primitive geometry
    ├── testDrakeSimConfig.cpp
    ├── testDrakeDriverLifecycle.cpp
    ├── testDrakeDriverPhysics.cpp
    └── testDrakeDriverFloatingBase.cpp
```

Plus an example at `motorium/examples/drake/` (section 8).

Namespace: `motorium::drake_sim` (avoid `motorium::drake`, which shadows Drake's own top-level `drake::` namespace inside motorium code).

## 5. Public API

### 5.1 `DrakeSimConfig`

```cpp
namespace motorium::drake_sim {

struct GroundConfig {
  bool enabled{true};
  double staticFriction{2.0};
  double dynamicFriction{1.5};
  double huntCrossleyDissipation{2.0};
  double pointStiffness{5.0e4};
};

struct DrakeSimConfig {
  std::string modelPath;                                  // URDF or SDF; must match RobotHAL's model
  std::shared_ptr<const model::RobotState> initStatePtr;  // nullptr -> zero config, root at z = 1 m
  // Applied from construction (and after reset) until the first setAction(); see section 6.6.
  std::shared_ptr<const model::RobotJointFeedbackAction> initialActionPtr;

  double dt{0.0005};
  double defaultJointDamping{10.0};
  bool addMissingActuators{true};         // URDF without <transmission>: one actuator per joint

  GroundConfig ground;
  bool disableSelfCollisions{true};

  // Bodies whose ground contact sets RobotState contact flag i. Empty -> flags are left untouched.
  std::vector<std::string> contactBodyNames;

  // Fault when the floating-base root height drops below this value. nullopt -> disabled.
  std::optional<double> faultBelowRootHeight;

  // Drake PackageMap entries for package:// URIs, plus an optional env var to scan (e.g. "AMENT_PREFIX_PATH").
  std::vector<std::pair<std::string, std::string>> packagePaths;
  std::optional<std::string> packagePathEnvVar;

  // Physics-thread scheduling. Empty / 0 -> leave as inherited.
  std::vector<int> physicsThreadCpus;
  int physicsThreadPriority{0};           // > 0 requests SCHED_FIFO; warns if not permitted

  bool headless{true};                    // false -> start DrakeMeshcatVisualizer
  double visualizationRateHz{30.0};
  std::string meshcatHost{"localhost"};
  std::optional<int> meshcatPort;

  bool verbose{false};

  void validate() const;                  // dt > 0, frictions >= 0, static >= dynamic, rate > 0, ...
};

}  // namespace motorium::drake_sim
```

Field naming follows `MujocoSimConfig` (camelCase fields).

Defaults: the ground and damping values above are the ones `drake_mrt_sim.launch.py` passes and that walked (friction 2.0/1.5, dissipation 2.0, stiffness 5e4, damping 10, dt 0.5 ms). The legacy `DrakeSimConfig` header defaults (friction 0.9/0.7) were never the tested values; don't copy them.

Contact flags: `RobotState::setConfigurationToZero()` sets every contact flag to `true`. Leaving `contactBodyNames` empty therefore reproduces the legacy `force_mujoco_contact_flags: true`, which the walking setup used.

### 5.2 `DrakeDriver`

```cpp
namespace motorium::drake_sim {

class DrakeDriver final : public hal::DriverBase {
 public:
  DrakeDriver(hal::HalKey key, const model::RobotDescription& description, const DrakeSimConfig& config);
  ~DrakeDriver() override;

  // Thread-safe copy of the latest physics snapshot, for visualizers and tests. Never blocks on a physics step.
  void copyStateSnapshot(model::RobotState& state) const;
  DrakeMetrics getMetrics() const;
  std::string getMeshcatUrl() const;      // empty when headless
  const DrakeSimConfig& getConfig() const { return config_; }

  // Advances exactly one step from the calling thread. Only valid while the physics thread is not running;
  // used by deterministic tests.
  void stepOnceForTesting();

 private:
  void startImpl() override;
  void stopImpl() override;
  void resetImpl() override;
  void updateStateImpl(model::RobotState& robot_state) override;
  void setActionImpl(const model::RobotJointFeedbackAction& action) override;

  void simulationLoop(std::stop_token st);
  void physicsStep() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_mutex_);
  void publishSnapshot() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_mutex_);
  void applyInitialState() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_mutex_);

  struct Impl;                            // all Drake types live here; keeps Drake out of the public header
  DrakeSimConfig config_;

  absl::Mutex action_mutex_;
  model::RobotJointFeedbackAction action_internal_ ABSL_GUARDED_BY(action_mutex_);
  bool action_received_ ABSL_GUARDED_BY(action_mutex_){false};

  mutable absl::Mutex sim_mutex_;         // plant context, simulator, scratch buffers
  std::unique_ptr<Impl> impl_ ABSL_PT_GUARDED_BY(sim_mutex_);

  mutable absl::Mutex snapshot_mutex_;    // held only for the copy, never across a physics step
  model::RobotState snapshot_ ABSL_GUARDED_BY(snapshot_mutex_);

  std::jthread physics_thread_;
  std::unique_ptr<DrakeMeshcatVisualizer> visualizer_;
};

}  // namespace motorium::drake_sim
```

`Impl` (in the `.cpp`) holds: `Diagram`, `Simulator`, plant pointer, `ModelInstanceIndex`, root `BodyIndex`, and the cached index maps from section 6.2.

## 6. Implementation details

### 6.1 Construction (port of `DrakeSimInterface::buildPlant`)

1. `config.validate()`.
2. `DiagramBuilder` + `AddMultibodyPlantSceneGraph(&builder, config.dt)`.
3. `set_discrete_contact_approximation(kSap)`, `set_contact_model(kPointContactOnly)` (legacy values).
4. `Parser` with `packagePaths` / `packagePathEnvVar`; `AddModels(modelPath)`; `MT_CHECK` exactly one model instance.
5. For each joint in `RobotDescription`:
   - `MT_CHECK` the joint exists in the plant and has exactly one DoF (revolute or prismatic, matching `RobotDescription`'s supported types).
   - `set_default_damping(defaultJointDamping)`.
   - If `addMissingActuators` and no actuator exists: `AddJointActuator(name, joint, torque_bounds.max)`.
6. `MT_CHECK` every actuator in the model instance maps to a `RobotDescription` joint, and the counts match.
7. Ground: legacy 20 × 20 × 0.2 m box at z = −0.1 with `AddContactMaterial(...)`, plus a visual. Keep the box for parity in the first version; consider `HalfSpace` later.
8. If `disableSelfCollisions`: `CollisionFilterDeclaration().ExcludeWithin(all geometries of the model instance)`.
9. Root body: `HasUniqueFloatingBaseBody(model)` → floating base via `GetUniqueFloatingBaseBodyOrThrow` (the `...FreeBase...` names are deprecated in Drake 1.52); otherwise fixed base. No hardcoded `"pelvis"`.
10. `Finalize()`, build the diagram, create the `Simulator` with `set_target_realtime_rate(0.0)` (pacing is ours).
11. Build index maps (6.2), apply the initial state, fix the actuation input port to zero (or the initial action), `Initialize()`, publish the first snapshot.
12. Create the visualizer if `!headless`.
13. `markReady()`.

Any failure throws or `MT_CHECK`s before `markReady()`, so a misconfigured driver never reaches `READY`.

### 6.2 Cached index maps (fixes legacy per-step name lookups)

Built once, stored as `std::vector<int>` / `std::vector<joint_index_t>`:

| Map | Source |
|---|---|
| `q_index[i]`, `v_index[i]` for robot joint `i` | `joint.position_start()`, `joint.velocity_start()` |
| `actuator_robot_index[k]` for actuator `k` (plant actuation order) | actuator's joint name → `RobotDescription::getJointIndex` |
| `actuator_q_index[k]`, `actuator_v_index[k]` | same as above, per actuator |
| `torque_bounds[k]` | `RobotDescription::getJointDescription(i).torque_bounds` |
| `contact_body_index[c]` | `contactBodyNames[c]` → `BodyIndex` |

Pre-allocated scratch: `Eigen::VectorXd actuation(num_actuators)`.

The physics loop reads `plant.GetPositionsAndVelocities(context)` (whole-plant overload, which returns a view, not a copy) and indexes it with the maps. **Verify in Drake 1.52 that the chosen accessors don't allocate**; the model-instance overloads do return copies.

### 6.3 Physics step

```
physicsStep():                                   [holds sim_mutex_]
  copy action_internal_ under action_mutex_ (short lock; FixedIDArray copy, no allocation)
  if !action_received_ and initialActionPtr: use *initialActionPtr
  x = plant.GetPositionsAndVelocities(context)
  for k in actuators:
      tau = action[actuator_robot_index[k]].getTotalFeedbackTorque(x[q_index], x[v_index])
      tau = isfinite(tau) ? torque_bounds[k].clamp(tau) : 0     # Drake does not clamp actuation itself
      actuation[k] = tau
  actuation_port.FixValue(context, actuation)
  simulator.AdvanceTo(t + dt)
  check state finite; check faultBelowRootHeight
      -> reportFault("..."); loop stops stepping
  publishSnapshot()
```

Using `getTotalFeedbackTorque()` keeps the semantics identical to `MujocoDriver`, including implicit-PD controllers (gains evaluated at 2 kHz in the driver).

### 6.4 State readout and frame conventions

`publishSnapshot()` fills `snapshot_` from the plant, then `updateStateImpl()` only copies `snapshot_` under `snapshot_mutex_`. The controller never waits on a 300–450 µs Drake step. (`MujocoDriver` holds its reader lock across `mj_step`; that's tolerable at 50–80 µs but not for Drake.)

Motorium `RobotState` conventions, which must match `MujocoDriver` exactly:

| Field | Convention | From Drake |
|---|---|---|
| root position | world frame | `EvalBodyPoseInWorld(root).translation()` |
| root orientation | rotation local → world | `EvalBodyPoseInWorld(root).rotation()` → quaternion |
| root linear velocity | **local** frame | `R⁻¹ · EvalBodySpatialVelocityInWorld(root).translational()` |
| root angular velocity | **local** frame | `R⁻¹ · ...rotational()` |
| joint q, v | per joint | via `q_index` / `v_index` |
| time | sim time | `context.get_time()` |
| contact flag c | body `c` in contact | `ContactResults` point pairs vs `contact_body_index` |

Setting state (`applyInitialState`) is the inverse: `SetFreeBodyPose`, `SetFreeBodySpatialVelocity` with velocities rotated into the world frame, joint q/v via `SetPositionsAndVelocities`.

A test (7.4) checks these conventions against an analytic case so MuJoCo and Drake agree.

### 6.5 Lifecycle mapping

| DriverBase hook | DrakeDriver behaviour |
|---|---|
| constructor | build everything, physics **not** stepping, `markReady()` |
| `startImpl()` | spawn `physics_thread_` (apply CPU affinity / priority if configured); start visualizer |
| `stopImpl()` | `request_stop()` + join physics thread (skip join if called from it); stop visualizer |
| `resetImpl()` | under `sim_mutex_`: restore initial state, zero actuation, clear `action_received_`, `Initialize()`, publish snapshot. No sleeps. Allowed while RUNNING (DriverBase permits it) |
| `updateStateImpl()` | copy snapshot |
| `setActionImpl()` | copy into `action_internal_`, set `action_received_` |

Fault handling:
- Root below `faultBelowRootHeight`, non-finite state, or a Drake exception in `AdvanceTo` → `reportFault(reason)`. The physics loop stops stepping immediately.
- `RobotHAL` sees `FAULT` on the next `updateState()`/`setAction()`, safety-stops all drivers, and latches.
- Application recovery: `hal.resetDrivers(); hal.startDrivers();`.

This replaces the legacy auto-reset in the physics thread, the 1 s `sleep` inside it, and `getFallResetCount()` polling.

Equivalence with the legacy fall handling, which the walking setup depended on:

| Legacy (`DrakeSimInterface` + `drake_mrt_sim_node`) | motorium |
|---|---|
| `resetUnlocked()` restores the initial state; actions become `q_des = init`, `qd_des = 0`, `ff = 0`, **kp/kd kept from the last action** | `resetImpl()` restores the initial state; `initialActionPtr` (hold pose with explicit gains) applies until the next `setAction()` |
| physics frozen for `reset_cooldown_s` (1 s) by a `sleep` in the physics thread | physics stopped between the fault and the app's `startDrivers()`, so the app controls the pause |
| node polls `getFallResetCount()`, then runs its MPC recovery while physics is frozen | app sees `hal.getState() == FAULT`, calls `resetDrivers()`, runs the same MPC recovery, then `startDrivers()` |

The motorium version keeps physics paused for exactly as long as the MPC recovery takes, instead of a fixed 1 s.

### 6.6 Holding the robot before the first action

`RobotHAL::setAction()` on a driver that is still `READY` returns false and triggers a safety stop. A controller therefore can't send a holding action before `startDrivers()`. Without one, the first physics steps run with zero torque and the robot sags. The legacy startup avoided this by applying a PD hold before starting physics.

Proposed solution, self-contained in this package: `DrakeSimConfig::initialActionPtr` is applied from construction and after every reset until the first `setAction()` arrives.

An alternative that would also fix `MujocoDriver` is allowing `setAction()` while `READY` in `DriverBase`/`RobotHAL`. That's a core API change and needs the maintainer's decision (section 11).

### 6.7 Timing, pacing and metrics

- Loop: `physicsStep()` → `nextWakeup += dt` → `sleep_until(nextWakeup)`. If more than one step late, resync to now and count an overrun (the legacy behaviour; no burst catch-up).
- `DrakeMetrics`: step compute time (mean / max over a window), real-time factor, overrun count, cumulative drift. Same spirit as `motorium_mujoco::Metrics`.
- `physicsThreadCpus` → `pthread_setaffinity_np`; `physicsThreadPriority > 0` → `SCHED_FIFO`, with a warning if not permitted (`ulimit -r` is 0 on default Ubuntu).

Background: the legacy Drake step takes 300–450 µs of the 500 µs budget, against 50–80 µs for MuJoCo. Thread placement decides whether real time holds, and the cached index maps should reduce the step cost.

### 6.8 Meshcat visualization (`DrakeMeshcatVisualizer`)

Port of legacy `buildVisualization` / `publishVisualization`:
- A separate zero-timestep `MultibodyPlant` + `SceneGraph` + `MeshcatVisualizer` (prefix `/robot`), so visualization never touches the physics context.
- Own thread at `visualizationRateHz`; reads `driver.copyStateSnapshot()` into a pre-allocated `RobotState`; sets the pose; `ForcedPublish`.
- Owned by the driver and created only when `!headless`, mirroring `MujocoRenderer`.
- Exposes `getMeshcatUrl()`.

## 7. Tests (gtest, `ament_add_gtest` + `cc_test`)

All tests use the primitive-geometry models in `test/resources/` (no meshes, fast to load), and `stepOnceForTesting()` for determinism where possible.

| Test file | Cases |
|---|---|
| `testDrakeSimConfig` | `validate()` rejects dt ≤ 0, negative friction, static < dynamic friction, rate ≤ 0 |
| `testDrakeDriverLifecycle` | constructs to `READY`; start/stop/start again; `reset` from READY/RUNNING/FAULT; `RobotHAL` safety-stops when the driver faults; joints missing from the model → construction fails |
| `testDrakeDriverPhysics` (fixed-base pendulum) | free swing period ≈ analytic within tolerance; implicit PD (`ImplicitJointPDController`) holds a setpoint; torque clamped to `torque_bounds`; NaN action → zero torque, no crash |
| `testDrakeDriverFloatingBase` (biped) | free fall: z(t) ≈ z₀ − ½gt²; root velocity frames: spin about world z, check local angular velocity sign and magnitude; stands under PD for 5 s sim time; contact flags true when standing; `faultBelowRootHeight` → `FAULT`; `resetDrivers()` restores the initial state |
| (optional) allocation check | run N steps under a counting allocator hook; expect zero allocations in `physicsStep` / `updateStateImpl` / `setActionImpl` |

Benchmark (not a pass/fail test): mean and max `physicsStep` time for G1 at dt = 0.5 ms, compared to the legacy figure of 300–450 µs.

## 8. Example

`motorium/examples/drake/g1_stand_drake.cpp` (and/or a smaller robot): `RobotHAL` + `DrakeDriver` (Meshcat on) + `ImplicitJointPDController` holding a standing pose. Loop structure identical to `examples/arx/arx_mujoco_sim.cpp`, so the only difference from the MuJoCo example is the driver.

Stretch: Drake's parser has partial MJCF support. Loading `examples/arx/model/arx_arm.xml` in Drake would give a same-model MuJoCo/Drake comparison. Check feasibility before relying on it.

## 9. Consumer integration in `wb_humanoid_mpc` (outside this package)

Listed here so the motorium work is scoped against what's needed for walking parity.

1. Bring `g1_29dof_drake.urdf` (+ `.xml`) from the legacy branch into `robot_models/unitree_g1/g1_description/urdf/`, verbatim. It's required: unlike `g1_29dof.urdf` (37 collision elements, mostly meshes), its only collision geometry is 4 spheres of 5 mm radius per foot. It has 29 joints with effort limits, no `<transmission>` tags (hence `addMissingActuators`) and no floating joint (Drake adds the free base).
2. **Stand, single process:** in `CentroidalMpcRobotSim.cpp`, select `MujocoDriver` or `DrakeDriver` by parameter.
3. Controller options in `CentroidalMpcMrtJointController` (config, defaults = current behaviour):
   - per-joint Kp/Kd from `task.info` (legacy `trackingKp/Kd`)
   - finite-difference `qdd_des` clamped to 35 rad/s²
   - joint-damping compensation in the feedforward torque
   - continuous yaw unwrapping in `updateMpcState`
4. Hold the `MRT_BASE&` in the controller (both `MPC_MRT_Interface` and `MRT_ROS_Interface` derive from it) so it runs in-process or against a separate MPC process.
5. **Walk, two processes:** new executable (MPC node + `MRT_ROS_Interface` + `RobotHAL`/`DrakeDriver`) with the legacy startup warm-up, fall recovery via `FAULT` → `resetDrivers()`, and the velocity-command bridge. Port `drake_mrt_sim.launch.py` with `taskset` pinning.
   - The legacy feedforward adds `joint_damping · qd_des` to cancel the simulator's joint damping. That couples the controller config to `DrakeSimConfig::defaultJointDamping`; keep the two values in one place (the launch file).
   - Keep the `[standing_telemetry]` / `[walking_telemetry]` log lines in the same format: the legacy Python tests parse them (step 7).
6. Later experiment: single process with thread affinity (MPC threads on one core set, `physicsThreadCpus` on another, `OMP_NUM_THREADS` limited), compared on the real-time factor.
7. **Acceptance tests:** port the legacy `robot_runtime/humanoid_drake_sim/tests/*.py` (11 files). They launch `drake_mrt_sim.launch.py` and parse the telemetry log lines, so they work almost unchanged if the launch-file name and log format stay. Fix the hardcoded `WORKSPACE_ROOT` and the process-kill patterns.

**Parity acceptance**, from the legacy `Documentation.md` §8.3 and `tests/test_*.py`:
- stands ≥ 15 s sim time, pelvis z ∈ [0.65, 0.90] m, |roll|, |pitch| ≤ 0.35 rad, 0 falls
- forward command v_x = 0.4 → Δx > 0.30 m, 0 falls
- backward command v_x = −0.4 → Δx < −0.20 m, 0 falls
- Meshcat renders the robot standing and walking

## 10. Build system

### Rule: Drake enters only through `find_package(drake CONFIG)` and `drake::drake`

No paths and no vendor names in `motorium_drake`'s CMake or code. Then only the environment changes between phases:

| Phase | Drake comes from | CMake finds it via | Runtime finds `libdrake.so` via | Change to `motorium_drake` |
|---|---|---|---|---|
| 1. Local (Ubuntu 24.04 + Jazzy) | `~/drake-build/install` | `CMAKE_PREFIX_PATH` in the shell | `LD_LIBRARY_PATH` in the shell | none |
| 2. `drake_vendor` | pinned `drake-<version>-noble.tar.gz` in the vendor's install prefix | vendor env hook prepends `CMAKE_PREFIX_PATH` | vendor env hook prepends `LD_LIBRARY_PATH` | add `<depend>drake_vendor</depend>` |
| 3. Docker (`ros:jazzy-ros-base`, Ubuntu 24.04) | same as phase 2 | same | same | none |

### Verified on the development machine (Drake `v1.52.0-94-g38121ab8`, GCC-built)

- `find_package(drake)` pulls Ubuntu 24.04's system Eigen 3.4.0, fmt 9.1 and spdlog 1.12, the same packages the official noble binaries require (`share/drake/setup/packages-noble-binary.txt`). The vendored tarball is therefore compatible with the local build.
- A minimal plant + simulator program builds and runs with GCC 13.3 / C++20 (what `wb_humanoid_mpc`'s `make build-all` uses) and with clang 19 / C++23 / `-Wthread-safety -Werror=thread-safety-analysis -Wall -Wextra -Wpedantic` (motorium's settings), with zero warnings. Imported-target includes are treated as system headers.
- **Installed binaries can't find `libdrake.so`**: CMake strips the build RPATH on install. Fix with `LD_LIBRARY_PATH` (phase 1: shell; phases 2–3: vendor env hook). Don't use `CMAKE_INSTALL_RPATH_USE_LINK_PATH`; it bakes the local install path into every binary.
- `drake-config.cmake` computes its prefix from its own location, so the extracted tarball can be copied anywhere.
- Drake's install has no `drake-config-version.cmake`, so `find_package(drake <version>)` can't enforce a minimum version. Document the tested version.
- `/usr/bin/clang` on this machine is clang 10, which can't compile C++23. Use `clang-19` explicitly for clang builds.

### Phase 1: local

```bash
# ~/.bashrc, after sourcing ROS
export DRAKE_INSTALL=$HOME/drake-build/install
export CMAKE_PREFIX_PATH=$DRAKE_INSTALL${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=$DRAKE_INSTALL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

```bash
# GCC build, as the full wb_humanoid_mpc build does it
make build-motorium_drake && make test-motorium_drake

# clang build for the thread-safety analysis (GCC ignores the annotations); separate build/install dirs
colcon build --packages-up-to motorium_drake --build-base build_clang --install-base install_clang \
  --cmake-args -DCMAKE_C_COMPILER=clang-19 -DCMAKE_CXX_COMPILER=clang++-19 \
  "-DCMAKE_CXX_FLAGS=-std=c++23 -Wthread-safety -Werror=thread-safety-analysis"
```

### colcon (`CMakeLists.txt`, `package.xml`)

- Dependencies: `motorium_core`, `motorium_model`, `motorium_hal`, `absl` (via `absl_vendor`), `Eigen3`, `drake`.
- Link Drake `PRIVATE` (Drake types stay behind `Impl`), but list it in `ament_export_dependencies(...)`: the library is static, so downstream executables still link `libdrake.so` and need `find_package(drake)`.
- `package.xml`: no `drake_ros`, and no `<depend>drake</depend>` (not a rosdep key; it would break `rosdep install`) until `drake_vendor` exists.
- **Skip when Drake is missing.** `wb_humanoid_mpc`'s Makefile builds every package under `lib/motorium/motorium` (`RUNTIME_PACKAGES`), and motorium's CI runs on amd64 and arm64. Without a guard, `make build-all` fails everywhere Drake isn't installed:
  ```cmake
  find_package(drake CONFIG QUIET)
  if(NOT drake_FOUND)
    message(WARNING "Drake not found; motorium_drake is skipped. Add a Drake install to CMAKE_PREFIX_PATH.")
    ament_package()
    return()
  endif()
  ```
  Downside: downstream `find_package(motorium_drake)` succeeds without targets. The alternative is a hard failure plus `--packages-skip motorium_drake` in CI. Maintainer decision (section 11).

### Phase 2: `drake_vendor`

- Location: `lib/drake_vendor/` in motorium, next to `mujoco_vendor` and `absl_vendor`. The Makefiles build with `--packages-up-to`, so it gets built automatically once `motorium_drake` depends on it.
- `ExternalProject_Add` with the release tarball URL and `URL_HASH SHA256` (pattern: `mujoco_vendor`). x86_64 only; fail clearly on aarch64 (no Ubuntu 24.04 arm64 binaries).
- `install(DIRECTORY ...)` the extracted tree into the vendor prefix; `lib/cmake/drake/drake-config.cmake` comes with it.
- `ament_environment_hooks` prepending `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH` (pattern: `absl_vendor_prefix_path.sh.in`). This replaces the phase-1 `~/.bashrc` lines.
- Option `DRAKE_VENDOR_USE_SYSTEM`: skip the download when `find_package(drake)` already succeeds, so the local source build keeps working.
- Runtime prerequisites: the packages in `share/drake/setup/packages-noble-binary.txt`, via `dependencies.txt` or the Dockerfile.
- Pin a tagged release ≥ v1.52.0. Before switching, build `motorium_drake` once against the extracted tarball to catch API differences from the local dev build.
- Alternative: Drake's apt repository (`drake-dev` → `/opt/drake`) in the Dockerfile, no vendor package. Less work, but the dependency lives outside colcon.

### Phase 3: Docker

The base image is Ubuntu 24.04 like the local machine, so the vendored tarball is the only difference. `make build-all`, `make test-all`, and expose port 7000 for Meshcat.

### Bazel (`BUILD.bazel`)

- Drake can be consumed as a Bazel module, but it's a very large dependency for motorium's CI.
- First version: targets tagged `manual` so `bazel build //motorium/...` skips them, with a documented local override to the Drake checkout. Full Bazel support after the maintainer decides how Drake enters `MODULE.bazel`.

### Platforms

- Since v1.59.0 Drake publishes Ubuntu 24.04 (Noble) binaries for both x86_64 and arm64; `drake_vendor` picks the archive from `CMAKE_SYSTEM_PROCESSOR`. Docker on Apple Silicon runs linux/arm64 and gets the arm64 archive.

## 11. Open questions for the maintainer

Raise these in a GitHub issue on `manumerous/motorium` before the first PR.

1. **Action before start (6.6):** accept `initialActionPtr` in the driver config, or change `DriverBase`/`RobotHAL` to allow `setAction()` while `READY`?
2. **Fall handling:** confirm a fall should be `reportFault()` + application-driven `resetDrivers()`, not auto-reset inside the driver (`MujocoDriver` has a `reset_requested_` hook).
3. **Optional dependency:** CMake guard vs hard failure + CI skip; how and whether Drake enters `MODULE.bazel`.
4. **Thread scheduling knobs** (`physicsThreadCpus`, `physicsThreadPriority`): driver-level, or a shared motorium utility usable by `MujocoDriver` too?
5. **Namespace / package name:** `motorium::drake_sim` / `motorium_drake` OK?

## 12. Milestones

Each milestone is one reviewable PR to the fork; upstream PRs can merge milestones.

| # | Deliverable | Done when |
|---|---|---|
| M0 | Upstream issue (section 11); test URDFs | maintainer feedback on the open questions |
| M1 | Package skeleton: CMake, `package.xml`, config + `validate()`, constructor builds the plant and reaches `READY` | `colcon build --packages-select motorium_drake` passes; config + construction tests green |
| M2 | Physics thread, action → torque with clamping, snapshot state readout, lifecycle hooks | lifecycle + fixed-base physics tests green; clang thread-safety clean |
| M3 | Floating base, ground, contact flags, `faultBelowRootHeight`, reset, `initialActionPtr` | floating-base tests green (free fall, frames, stand 5 s, fault, reset) |
| M4 | `DrakeMeshcatVisualizer` + example | example shows the robot standing in Meshcat |
| M5 | Performance: index maps verified allocation-free, metrics, CPU affinity / priority | G1 step-time benchmark recorded; ≤ legacy 300–450 µs |
| M6 | Build/CI: guard or skip, `BUILD.bazel` (`manual`), `make format`, README replacing this plan | motorium CI green with and without Drake installed |
| M7 | `wb_humanoid_mpc`: G1 stands in single process via driver swap | standing criterion (section 9) met |
| M8 | `wb_humanoid_mpc`: two-process walking executable + launch | all parity criteria (section 9) met |
| M9 | Upstream PR to `manumerous/motorium`; bump submodule in `wb_humanoid_mpc` | merged |

## 13. Risks

| Risk | Mitigation |
|---|---|
| Drake step too slow for real time at dt = 0.5 ms | cached indices (M5), thread pinning, two-process layout; measure before tuning |
| Behaviour differs from legacy after refactor (e.g. ground shape, frames) | port physics parameters 1:1 first; frame-convention test; compare against legacy runs before changing anything |
| Drake's bundled third-party libraries clash with motorium's absl / Eigen | legacy linked Drake and absl together successfully; build the example early (M1/M2) to surface link issues |
| Upstream doesn't want a heavy optional dependency | settle M0 questions first; keep the package fully isolated so it can live in the fork if needed |
| Drake API changes between versions | document the tested version; isolate Drake calls in `DrakeDriver.cpp` behind `Impl` |

## 14. Legacy code reuse map

Paths relative to the legacy `robot_runtime/humanoid_drake_sim/`. Line counts include comments and blank lines.

### `src/DrakeSimInterface.cpp` (532 lines) → `motorium_drake`

| Legacy function (lines) | Reuse | Destination | Changes |
|---|---|---|---|
| `buildPlant` (117–186) | copy | `DrakeDriver` constructor (6.1) | effort limits from `torque_bounds`; one-DoF check instead of `RevoluteJoint` only |
| `buildVisualization` (187–237) | copy | `DrakeMeshcatVisualizer` (6.8) | none of substance |
| `setSimState` (332–364) | copy | `applyInitialState` (6.4) | name lookups OK here (init/reset only) |
| `updateThreadSafeRobotState` (365–415) | logic kept | `publishSnapshot` (6.4) | cached q/v indices; contact by `BodyIndex`; writes `snapshot_` |
| `setupJointIndexMaps` (291–331) | extend | index maps (6.2) | add q/v offsets per joint and per actuator |
| `configurePackageMap` (40–55) | ~6 lines | constructor | drop the `ament_index` / `g1_description` lookup |
| `simulationStep` (440–489) | torque loop kept | `physicsStep` (6.3) | `qd_des` → `v_des`; fall → `reportFault`; no `sleep` |
| `publishVisualization` (238–269) | adapt | visualizer thread | cached indices; pre-allocated state |
| `resetUnlocked` / `reset` (416–439) | adapt | `resetImpl` (6.5) | `initialActionPtr` instead of patching the last action |
| `Impl`, ctor, dtor (56–116) | ~half | `Impl`, constructor | `markReady()`; `std::jthread` |
| `simulationLoop` (490–511) | rewrite | `simulationLoop` (6.7) | `stop_token`, metrics, affinity |
| `initSim`, `startSim`, `getFallResetCount`, `getVisualizationStatus` (512–532, 274–290) | drop | — | `DriverBase` lifecycle and `FAULT` |

Roughly 250 lines copied, 150 edited, 130 dropped or rewritten. New code motorium requires (DriverBase glue, absl locking, snapshot, build files, tests): roughly 300–400 lines.

### `include/humanoid_drake_sim/DrakeSimInterface.h` (94 lines)

Config fields (~25 lines) → `DrakeSimConfig` (5.1), with the launch-file values as defaults. Class declaration rewritten (5.2).

### `src/drake_mrt_sim_node.cpp` (845 lines) → `wb_humanoid_mpc` only (section 9), nothing into motorium

| Part (lines) | Destination |
|---|---|
| `makeInitialState` with 1.43 mm grounding offset (44–71) | consumer, passed as `initStatePtr` |
| `ContinuousYawUnwrapper`, `updateMpcState`, `updateMpcObservation` (74–150) | `CentroidalMpcMrtJointController` (yaw unwrapping is missing on main) |
| `trackingKp` / `trackingKd` (154–184) | `task.info` per-joint gains |
| ROS parameters (204–250) | launch file |
| velocity bridge (279–405) | consumer node / command filter |
| startup steps 1–5 (407–556) | consumer startup sequence; step 1 maps to "driver READY, not started" + `initialActionPtr` |
| fall recovery (589–710) | consumer, triggered by `FAULT` (6.5 table) |
| 500 Hz control computation (712–794) | merge into `CentroidalMpcMrtJointController` (finite-difference `qdd`, damping compensation) |
| telemetry + visualization (796–840) | consumer; keep the log format for the Python tests |

### Other legacy files

| File | Reuse |
|---|---|
| `robot_models/unitree_g1/g1_description/urdf/g1_29dof_drake.urdf` + `.xml` | verbatim, required (section 9 step 1) |
| `robot_models/unitree_g1/g1_centroidal_mpc/launch/drake_mrt_sim.launch.py` | copy; change package/executable |
| `tests/*.py` (11 files, ~2900 lines) | acceptance tests, almost unchanged (section 9 step 7) |
| `src/drake_urdf_interface_smoke_test.cpp` | template for the first construction gtest |
| `src/gravity_comp_hold_test.cpp`, `src/drake_sap_stand_test.cpp` | reference for the standing test; code not reusable (own plants) |
| `src/drake_sim_node.cpp`, `src/policy_to_command_node.cpp`, other diagnostics | discard (abandoned design / own plants) |
| `CMakeLists.txt` | only the `find_package(drake)` / link lines |
| `Documentation.md` | reference only; gains in §2.2 and the OpenMP explanation are out of date or unverified |

### Checklist

- [ ] `buildPlant` → constructor (6.1); drop hardcoded `g1_description`; root body auto-detected
- [ ] `setupJointIndexMaps` → index maps incl. q/v offsets (6.2); torque limits from `RobotDescription`
- [ ] `simulationStep` → `physicsStep` (6.3); `qd_des` → `v_des`; no `GetJointByName` in the loop
- [ ] `updateThreadSafeRobotState` → `publishSnapshot` (6.4); contact bodies by `BodyIndex`
- [ ] `setSimState` → `applyInitialState` (6.4)
- [ ] `resetUnlocked` / auto-reset / `sleep` → `resetImpl` + `reportFault` (6.5)
- [ ] `simulationLoop` (`std::thread` + atomic flag) → `std::jthread` + `stop_token` (6.7)
- [ ] `std::mutex` → `absl::Mutex` with annotations
- [ ] `buildVisualization` / `publishVisualization` → `DrakeMeshcatVisualizer` (6.8)
- [ ] Defaults from `drake_mrt_sim.launch.py`, not the legacy header (5.1)
- [ ] Remove `drake_ros` dependency

## 15. Implementation status and findings

[README.md](README.md) documents the implemented package and is the reference for its API; sections 5–6 above are the original design and differ in the details listed under "Deviations from sections 5–6".

### Done

| Item | Files |
|---|---|
| `DrakeSimConfig` + `validate()` | `include/motorium_drake/DrakeSimConfig.h`, `src/DrakeSimConfig.cpp` |
| `DrakeDriver`: HAL lifecycle, physics thread, state snapshot, faults, initial action, metrics | `include/motorium_drake/DrakeDriver.h`, `src/DrakeDriver.cpp` |
| `DrakeSimulation` (private): Drake plant, index maps, motorium ⇄ Drake conversion | `src/DrakeSimulation.h`, `src/DrakeSimulation.cpp` |
| `DrakeMeshcatVisualizer` | `include/motorium_drake/DrakeMeshcatVisualizer.h`, `src/DrakeMeshcatVisualizer.cpp` |
| Model loading helpers (private) | `src/DrakeModelLoading.h` |
| colcon build; Drake required, provided by `drake_vendor` (v1.59.0 prebuilt, x86_64 + arm64) | `CMakeLists.txt`, `package.xml`, `../../lib/drake_vendor/` |
| Documentation | `README.md` |
| 32 gtest cases in 5 executables, test URDFs | `test/` |

Verified with Drake `v1.52.0-94-g38121ab8`: all tests pass with GCC 13.3 / C++20 (`make build-motorium_drake && make test-motorium_drake`) and with clang 19 / C++23 / `-Werror=thread-safety-analysis`; 10 repeated runs without a failure; `clang-format-18` clean.

### Deviations from sections 5–6

Simplified to what current uses and tests need:
- Removed until a real use needs them: `physicsThreadCpus` / `physicsThreadPriority` (thread pinning can be done with `taskset` for now), `packagePathEnvVar` (use `packagePaths`), the `addMissingActuators` and `disableSelfCollisions` switches (both behaviours are always on), `stepOnceForTesting()` (replaced by `manualStepping` + `advance()`).
- The Drake side lives in a private class `DrakeSimulation` instead of an anonymous `Impl` struct; `DrakeDriver` only handles threads, locks, lifecycle and faults.
- No double-buffered staging state: the physics thread writes the snapshot directly under `snapshot_mutex_` (held for microseconds).
- `DrakeMetrics`: step count, mean / max step time, overrun count. Not reset by `resetDrivers()`. With `verbose`, the physics thread prints the real-time factor and these metrics every 5 s.
- `DrakeMetrics.h` merged into `DrakeDriver.h`.

### Not done yet

- Example in `motorium/examples/drake/` (M4).
- `BUILD.bazel` (M6).
- G1 step-time benchmark and allocation check (M5).
- Two-process walking setup in `wb_humanoid_mpc` (M8). The single-process sim (`simulator:=drake`, `drake_sim.launch.py`) stands and walks.
- Testing the `drake_vendor` build in the Docker image.

### Additions beyond sections 5–6 (kept)

- `DrakeSimConfig::manualStepping` + `DrakeDriver::advance(n)`: no physics thread; the simulation advances only when called. Makes tests deterministic and lets them run faster than real time. Requires the driver to be `RUNNING`, so it works with `RobotHAL::setAction()` / `updateState()`.
- The applied (clamped) torque is reported as `RobotState` joint effort.
- `modelPath` defaults to the `RobotDescription`'s model path.

### Findings

1. **Step-time drift bug, also in the legacy code.** `AdvanceTo(get_time() + dt)` accumulates rounding error. Within about 10 steps the target lands just past Drake's next update time, after which one call performs two discrete updates and the state stays one step (0.5 ms) ahead of its reported time. Verified with a standalone probe: 1001 updates after 1000 calls. Fixed by advancing to `(n + 1) * dt` with an integer step counter. The legacy `DrakeSimInterface::simulationStep` has the same bug. Caught by `FreeFallFollowsBallisticMotion` and `RootLinearVelocityIsInTheLocalFrame`.
2. **GCC/clang ABI mismatch with a GCC-built Drake.** Drake instantiates its body collection with the alias template `Link = RigidBody`; GCC mangles it as `RigidBody`, clang 19 as `Link`. Clang-compiled calls to `MultibodyPlant::get_body(BodyIndex)` therefore fail to link against a GCC-built `libdrake.so`. The driver avoids `get_body()` and reaches bodies through joints (`src/DrakeModelLoading.h`). Any future code compiled with clang should do the same, or Drake must be built with the same compiler. Relevant for motorium's clang CI once Drake is available there.
3. **Meshcat is single-thread.** Drake requires all calls on a `Meshcat` instance from the thread that created it, so the visualizer builds its scene and server inside its own thread.
4. **Drake 1.52 API changes:** include `drake/multibody/math/spatial_algebra.h`, not `spatial_velocity.h`; `HasUniqueFreeBaseBody` / `GetUniqueFreeBaseBodyOrThrow` are deprecated in favour of `...FloatingBaseBody...`.
5. **Default joint damping of 10 makes light links heavily overdamped.** For the 1 kg, 0.5 m test pendulum the time constant is about 2 s, so tests that need free motion set `defaultJointDamping = 0`. Not a driver issue, but relevant when reusing the G1 defaults on other robots.
