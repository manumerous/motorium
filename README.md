# Motorium

![Bazel Build](https://github.com/manumerous/motorium/actions/workflows/bazel_build_test.yml/badge.svg)
![Colcon Build](https://github.com/manumerous/motorium/actions/workflows/colcon_build_test.yml/badge.svg)
![Style Check](https://github.com/manumerous/motorium/actions/workflows/format_test.yml/badge.svg)

Motorium is a high-performance, realtime-safe framework for robot control, designed to allow rapid prototyping of robot controllers across hardware and simulators. It prioritizes deterministic execution, cache locality, and type safety.

## Key Design Principles
Robot data must often be accessed in different subsets, orders, and formats across a robotics stack to satisfy the requirements of various components and libraries (controllers, simulators, drivers etc.). Motorium addresses this by storing robot data in a centralized, real-time–safe structure and exposing it through uniquely assigned integer IDs (e.g., for joints and sensors) for seamless integration across simulation and hardware.

### Realtime Data Structures
At the core of Motorium is the **`FixedIDArray`**, a lightweight, fixed-size container optimized for realtime control loops.
- **Zero Allocations**: All storage has to be allocated at construction time.
- **Dense Indexing**: Uses dense, zero-based integer IDs for fast deterministic access.
- **Cache Friendly**: Data is stored contiguosly, ensuring optimal cache coherence.
- **Eigen Interopability**: Seamlessly extracts data into Eigen vectors for math-heavy control logic without copying overhead where possible.

### Construction Time Name-to-Index Mapping
For humans, configuring robot control systems via names is convenient, butString lookups are expensive and non-deterministic. Motorium solves this by resolving all symbolic names to unique integer indices at **construction time**.
- **`RobotDescription`**: Builds an immutable mapping of Joint Names $\to$ unique joint indices based on a robot model (e.g. URDF).
- **Construction-Time Resolution**: Drivers and Controllers use the `RobotDescription` to resolve names to indices at construction time.
- **Runtime Efficiency**: The control loop operates exclusively on indices, guaranteeing safe, bounds-checked, and instant access to system state.

### Hardware Abstraction Layer (HAL)
The modular HAL allows the same controller code to run on simulated and real hardware without modification.
- **`RobotHardware`**: The central interface exposing `RobotState` and `RobotJointAction` to the control loop.
- **`DriverBase`**: Abstract base class for implementing specific backend drivers (e.g., MuJoCo simulation, EtherCAT masters, CAN bus interfaces).
