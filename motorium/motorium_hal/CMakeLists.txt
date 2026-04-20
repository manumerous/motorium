cmake_minimum_required(VERSION 3.5)
project(motorium_hal)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(motorium_model REQUIRED)
find_package(magic_enum REQUIRED)

find_package(motorium_core REQUIRED)
find_package(GTest REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
)

set(dependencies
        ament_cmake
        ament_index_cpp
        motorium_core
        motorium_model
        magic_enum
)

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/
)

# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_driver_base test/testDriverBase.cpp)
  target_include_directories(test_driver_base PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_driver_base motorium_model motorium_core magic_enum)
endif()

ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
