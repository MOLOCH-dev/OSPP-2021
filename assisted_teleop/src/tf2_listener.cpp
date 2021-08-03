#include <math.h>
#include <iostream>
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/time_source.hpp>
#include "geometry_msgs/msg/pose_stamped.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

class VelSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("velsubscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
    

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.c_str());
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_pub");
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("tf_pub", 10);
  float time = 0.5;
  

  //printf("TimePoint %d",tp);
//  std::shared_ptr<rclcpp::Clock> clock_ptr;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource timesource;
  timesource.attachClock(clock);

//  tf2_ros::Buffer tfBuffer(clock_ptr);
  std::unique_ptr<tf2_ros::Buffer> buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer);


  tf2::Transform transform_;
  transform_.setIdentity();

  geometry_msgs::msg::PoseStamped message;
  rclcpp::WallRate loop_rate(500);

  while (rclcpp::ok()) {

    try {

        geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform("odom", "base_footprint", builtin_interfaces::msg::Time());
        message.pose.position.x = transformStamped.transform.translation.x;
        message.pose.position.y = transformStamped.transform.translation.y;
        //RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", str(message.pose));
        std::cout<< message.pose.position.x <<std::endl;
        publisher->publish(message);
    }
    catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(node->get_logger(), "StaticLayer: %s", ex.what());
    }


    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
