cmake_minimum_required(VERSION 3.16)
project(absl_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)
include(GNUInstallDirs)

set(ABSL_TAG "20250814.1")

# We install Abseil INTO this vendor package's install prefix so it gets overlaid by colcon.
set(ABSL_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

ExternalProject_Add(absl_src
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG        ${ABSL_TAG}
  GIT_SHALLOW    TRUE
  UPDATE_COMMAND ""

  # Build in-source is off; we use a separate build directory.
  SOURCE_DIR "${CMAKE_BINARY_DIR}/absl-src"
  BINARY_DIR "${CMAKE_BINARY_DIR}/absl-build"

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${ABSL_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS=ON
    -DABSL_ENABLE_INSTALL=ON
    -DABSL_BUILD_TESTING=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON

  # Helps with multi-config generators; harmless otherwise
  INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
)

# --- Environment hook ---
# Ensure downstream packages can find abslConfig.cmake via CMAKE_PREFIX_PATH.
# ament will source this when you `source install/setup.bash`.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/absl_vendor_prefix_path.sh.in")

ament_package()
