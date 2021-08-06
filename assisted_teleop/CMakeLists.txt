cmake_minimum_required(VERSION 3.5)
project(assisted_teleop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs geometry_msgs)
add_executable(assisted_teleop_node src/assisted_teleop.cpp)
ament_target_dependencies(assisted_teleop_node rclcpp std_msgs geometry_msgs tf2 tf2_ros)
add_executable(footprint_sub_node src/footprint_sub.cpp)
ament_target_dependencies(footprint_sub_node rclcpp std_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS
  talker
  listener
  footprint_sub_node
  assisted_teleop_node
  DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
 )

ament_package()
