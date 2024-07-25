cmake_minimum_required(VERSION 3.8)
project(sample_contact_walking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ------- Fetch the Obelisk library ------- #
include(FetchContent)
FetchContent_Declare(
    Obelisk
    SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us Get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# ------- Fetch TORC ------- #
set(BUILD_WITH_PINOCCHIO ON CACHE BOOL "t")
set(BUILD_WITH_IPOPT OFF CACHE BOOL "t")
set(BUILD_WITH_OSQP ON CACHE BOOL "t")
set(BUILD_WITH_CLARABEL OFF CACHE BOOL "t")

FetchContent_Declare(
    TORC
    SOURCE_DIR $ENV{HOME}/torc
)

FetchContent_MakeAvailable(TORC)

# ------- Pinocchio ------- #
find_package(pinocchio 3.0.0 REQUIRED)

# ------- Executables ------- #
include_directories(include)
ament_export_include_directories(include)

add_executable(ll_controller src/controller_spin.cpp)
target_link_libraries(ll_controller PUBLIC Obelisk::Core Obelisk::Zoo)

add_executable(state_estimator 
    src/achilles_estimator.cpp
    )
target_link_libraries(state_estimator PUBLIC Obelisk::Core pinocchio::pinocchio)

ament_target_dependencies(
    state_estimator PUBLIC
    tf2_ros
    tf2
)

# ------- Install ------- #
install(TARGETS
    ll_controller
    state_estimator
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
