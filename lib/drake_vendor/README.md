# Drake vendor package

Installs the prebuilt [Drake](https://drake.mit.edu) release `v1.59.0` for Ubuntu 24.04 (noble, the
base of ROS 2 Jazzy) into this package's install prefix. `motorium_drake` depends on it.

- **Architecture**: `drake-1.59.0-noble.tar.gz` on x86_64 (Intel / AMD),
  `drake-1.59.0-noble-arm64.tar.gz` on aarch64 (ARM, including Docker on Apple Silicon).
  Each download is checked against its SHA256.
- **Finding Drake**: after `source install/setup.bash`, an environment hook adds this package to
  `CMAKE_PREFIX_PATH`, so `find_package(drake CONFIG)` works, and `lib/` (with `libdrake.so`) is on
  `LD_LIBRARY_PATH`.
- **System packages**: the prebuilt library links against Ubuntu's Eigen, fmt, spdlog, BLAS, LAPACK
  and gfortran, and loads EGL / X11 for rendering. The Docker image installs them via
  `dependencies.txt`; the full list is in `share/drake/setup/ubuntu/packages-noble-binary.txt` of the
  installed tree. The Python packages in that list are only needed for `pydrake`.
- **Updating**: change `DRAKE_VERSION` and both `DRAKE_SHA256` values in `CMakeLists.txt`. The hashes
  are the `.sha256` files published next to each archive on the
  [releases page](https://github.com/RobotLocomotion/drake/releases).
