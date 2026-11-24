cmake_minimum_required(VERSION 3.16)
project(drake_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

# Prebuilt Drake for Ubuntu 24.04 (noble), the base of the ROS 2 Jazzy images.
set(DRAKE_VERSION "1.59.0")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(DRAKE_ARCHIVE "drake-${DRAKE_VERSION}-noble-arm64.tar.gz")
  set(DRAKE_SHA256 "5c071e4d9b5591ab925833507096604c755c6098e545239d9c8ca4d8fd664dde")
else()
  set(DRAKE_ARCHIVE "drake-${DRAKE_VERSION}-noble.tar.gz")
  set(DRAKE_SHA256 "97c50c9e22a585cc5997cad22c99aadd58a21e3bcab5668000abf423d3965eba")
endif()

# Drake's CMake config locates its files relative to itself, so the extracted release is copied
# as is into this package's install prefix (bin/, include/, lib/, share/).
ExternalProject_Add(drake_prebuilt
  URL "https://github.com/RobotLocomotion/drake/releases/download/v${DRAKE_VERSION}/${DRAKE_ARCHIVE}"
  URL_HASH SHA256=${DRAKE_SHA256}
  PREFIX "${CMAKE_BINARY_DIR}/drake_prebuilt"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR> "${CMAKE_INSTALL_PREFIX}"
)

# Lets find_package(drake) find lib/cmake/drake/drake-config.cmake once install/setup.bash is
# sourced. ament_package() already adds lib/ to LD_LIBRARY_PATH, where libdrake.so lives.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/drake_vendor_prefix_path.sh.in")

ament_package()
