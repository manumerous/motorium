cmake_minimum_required(VERSION 3.5)
project(motorium_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(motorium_model REQUIRED)
find_package(absl CONFIG REQUIRED)

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
        absl
)

#############
## Library ##
#############

add_library(${PROJECT_NAME}
        src/ImplicitJointPDController.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
        absl::flat_hash_map
        absl::log
        absl::check
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ############
# # Testing ##
# ############

if (BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

        ament_add_gtest(test_implicit_pd_controller test/testImplicitJointPDController.cpp)
        target_link_libraries(test_implicit_pd_controller
                ${PROJECT_NAME}
        )

endif ()

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
