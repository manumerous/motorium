cmake_minimum_required(VERSION 3.16)
project(mujoco_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

# Set the Mujoco version and download details
set(MUJOCO_VERSION "3.15.0") 
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(MUJOCO_ARCH "aarch64")
  set(MUJOCO_SHA256 "fd7962da3a03c7e9ee25601e9890f36d9e67c265b4f8e00db1613aca913c4864")
else()
  set(MUJOCO_ARCH "x86_64")
  set(MUJOCO_SHA256 "319943b332fd84c968f655f695d8f4b03cc6a68f658bacd0591f0a552744e497")
endif()
set(MUJOCO_DOWNLOAD_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz")
set(MUJOCO_TARBALL "${CMAKE_BINARY_DIR}/mujoco_prebuilt/src/mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz")
set(MUJOCO_INSTALL_DIR ${CMAKE_BINARY_DIR}/mujoco) 

# Add Mujoco as an external project
ExternalProject_Add(mujoco_prebuilt
    URL ${MUJOCO_DOWNLOAD_URL}
    URL_HASH SHA256=${MUJOCO_SHA256}
    PREFIX ${CMAKE_BINARY_DIR}/mujoco_prebuilt
    INSTALL_DIR ${MUJOCO_INSTALL_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND tar --strip-components=1 -xzf ${MUJOCO_TARBALL} -C ${MUJOCO_INSTALL_DIR}
)

# Create an interface library for linking with Mujoco
add_library(mujoco INTERFACE)
add_dependencies(mujoco mujoco_prebuilt)

# Specify include directories and link libraries for Mujoco
target_include_directories(mujoco INTERFACE
    ${MUJOCO_INSTALL_DIR}/include
)
target_link_libraries(mujoco INTERFACE
  ${MUJOCO_INSTALL_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}
)

# Install Mujoco headers and library
install(DIRECTORY ${MUJOCO_INSTALL_DIR}/include/
        DESTINATION include)

install(FILES ${MUJOCO_INSTALL_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}
        DESTINATION lib)

# Create a symlink to the versioned target 
install(CODE "
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink libmujoco.so.${MUJOCO_VERSION} libmujoco.so
        WORKING_DIRECTORY \${CMAKE_INSTALL_PREFIX}/lib
    )
")
      
# Export Mujoco as a dependency
ament_export_include_directories(${MUJOCO_INSTALL_DIR}/include)
ament_export_libraries(mujoco)
ament_package()
