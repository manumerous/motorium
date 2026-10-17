cmake_minimum_required(VERSION 3.5)
project(motorium_core)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(absl CONFIG REQUIRED)
find_package(magic_enum REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        $<TARGET_PROPERTY:magic_enum::magic_enum,INTERFACE_INCLUDE_DIRECTORIES>
)

set(dependencies
        ament_cmake
        absl
        magic_enum
)

#############
## Library ##
#############

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
        absl::flat_hash_map
        absl::log
        absl::check
        magic_enum::magic_enum
)
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ############
# # Testing ##
# ############

if (BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

endif ()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/
)

# install(DIRECTORY launch rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
