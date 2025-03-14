cmake_minimum_required(VERSION 3.8)
project(sample_contact_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ------- Messages --------- #
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContactInfo.msg"
  "msg/ContactPolytope.msg"
  "msg/ContactSchedule.msg"
  "msg/CommandedTarget.msg"
  "msg/ContactPolytopeArray.msg"
  DEPENDENCIES
  std_msgs
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  std_msgs
  rosidl_default_generators
)
include_directories(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
