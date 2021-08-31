#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_assisted_teleop/assisted_teleop.hpp"
// #include "assisted_teleop.cpp"

using std::placeholders::_1;





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_node");
  nav2_assisted_teleop::AssistedTeleop <rclcpp::Node> myobject (node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}