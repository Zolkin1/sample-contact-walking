cmake_minimum_required(VERSION 3.8)
project(sample_contact_walking)

set(CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sample_contact_msgs REQUIRED)
# find_package(obelisk_estimator_msgs REQUIRED)

# ------- Fetch the Obelisk library ------- #
include(FetchContent)
FetchContent_Declare(
    Obelisk
    SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us Get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# ------- Fetch TORC ------- #
FetchContent_Declare(
    TORC
    SOURCE_DIR $ENV{HOME}/torc
)

FetchContent_MakeAvailable(TORC)

message(STATUS TORC_)

# ------- Pinocchio ------- #
find_package(pinocchio 3.0.0 REQUIRED)

# ------- Executables ------- #
include_directories(include)
ament_export_include_directories(include)

# ------- Controller ------- #
add_executable(ll_controller src/robot_mpc.cpp)
target_link_libraries(ll_controller PUBLIC Obelisk::Core Obelisk::Zoo Mpc StepPlanning Wbc) # Wbc

ament_target_dependencies(ll_controller
    PUBLIC
    visualization_msgs
    tf2_ros
    tf2
    sample_contact_msgs
    sensor_msgs)

# ------- Basic Test Controller ------- #
add_executable(basic_torque_control src/torque_control_test.cpp)
target_link_libraries(basic_torque_control PUBLIC Obelisk::Core Obelisk::Zoo) # Wbc

ament_target_dependencies(basic_torque_control
    PUBLIC
    visualization_msgs
    tf2_ros
    tf2
    sample_contact_msgs
    sensor_msgs)

# ------- Estimator ------- #
add_executable(state_estimator 
    src/robot_estimator.cpp
    )
target_link_libraries(state_estimator PUBLIC Obelisk::Core pinocchio::pinocchio StateEstimation Mpc Models)

ament_target_dependencies(
    state_estimator PUBLIC
    tf2_ros
    tf2
)

# ------- Simulation ------- #
add_executable(simulation src/robot_sim.cpp)
target_link_libraries(simulation PUBLIC Obelisk::Core)

# ------- Install ------- #
install(TARGETS
    ll_controller
    basic_torque_control
    state_estimator
    simulation
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
