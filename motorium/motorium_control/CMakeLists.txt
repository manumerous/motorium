cmake_minimum_required(VERSION 3.5)
project(motorium_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(motorium_model REQUIRED)

find_package(GTest REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
        ament_cmake
        ament_index_cpp
        motorium_model
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
