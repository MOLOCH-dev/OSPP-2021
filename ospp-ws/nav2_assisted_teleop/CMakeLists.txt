cmake_minimum_required(VERSION 3.5)
project(nav2_assisted_teleop)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_tools REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav2_msgs
  nav2_util
  nav2_common
  tf2
  tf2_ros
)

set(library_name ${PROJECT_NAME}_core)
set(exec_name ${PROJECT_NAME}_exec)

add_library(${library_name} SHARED
  src/assisted_teleop.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(${exec_name} src/main.cpp)



ament_target_dependencies(${exec_name}
  ${dependencies}
)

target_link_libraries(${exec_name}
  ${library_name}
)



install(TARGETS
  ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()

