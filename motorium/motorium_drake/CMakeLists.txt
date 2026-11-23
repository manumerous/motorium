cmake_minimum_required(VERSION 3.16)
project(motorium_drake)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

# Drake comes from drake_vendor, whose environment hook puts it on CMAKE_PREFIX_PATH.
find_package(drake CONFIG REQUIRED)
message(STATUS "${PROJECT_NAME}: using Drake from ${drake_DIR}")

find_package(motorium_core REQUIRED)
find_package(motorium_model REQUIRED)
find_package(motorium_hal REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#############
## Library ##
#############

add_library(${PROJECT_NAME}
  src/DrakeSimConfig.cpp
  src/DrakeDriver.cpp
  src/DrakeSimulation.cpp
  src/DrakeMeshcatVisualizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    motorium_hal::motorium_hal
    motorium_model::motorium_model
    motorium_core::motorium_core
    absl::synchronization
    absl::log
    absl::check
    Eigen3::Eigen
  # Drake types never appear in the public headers.
  PRIVATE
    drake::drake
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include/)

#############
## Testing ##
#############

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(motorium_control REQUIRED)

  function(motorium_drake_add_test name source)
    ament_add_gtest(${name} ${source} TIMEOUT 300)
    target_link_libraries(${name} ${PROJECT_NAME} motorium_control::motorium_control)
    target_compile_definitions(${name} PRIVATE TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
  endfunction()

  motorium_drake_add_test(test_drake_sim_config test/testDrakeSimConfig.cpp)
  motorium_drake_add_test(test_drake_driver_lifecycle test/testDrakeDriverLifecycle.cpp)
  motorium_drake_add_test(test_drake_driver_physics test/testDrakeDriverPhysics.cpp)
  motorium_drake_add_test(test_drake_driver_floating_base test/testDrakeDriverFloatingBase.cpp)
  motorium_drake_add_test(test_drake_meshcat_visualizer test/testDrakeMeshcatVisualizer.cpp)
endif ()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
# The library is static, so consumers still link libdrake.so and need find_package(drake).
ament_export_dependencies(motorium_core motorium_model motorium_hal absl Eigen3 drake)
ament_package()
