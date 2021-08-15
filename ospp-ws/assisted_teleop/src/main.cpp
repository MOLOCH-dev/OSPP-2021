// Copyright 2021 Anushree Sabnis

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <rclcpp/time_source.hpp> //NOLINT
#include <tf2_ros/transform_listener.h> //NOLINT
#include <tf2/utils.h> //NOLINT
#include "tf2_ros/buffer.h" //NOLINT
#include "assisted_teleop/assisted_teleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
  geometry_msgs::msg::PoseStamped out_pose_2;
  geometry_msgs::msg::PoseStamped global_pose_;
  geometry_msgs::msg::PolygonStamped::SharedPtr projected_footprint;
  double yaw_;
  double speed_;
  double projection_time = 1.5;

public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_f_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "local_costmap/published_footprint",
      10,
      std::bind(&MinimalSubscriber::footprint_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "local_costmap/projected_footprint", 10);
  }

private:
  std::unique_ptr<tf2_ros::Buffer> buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*buffer);
  geometry_msgs::msg::PoseStamped out_pose;
  void
  footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr footprint)
  {
    projected_footprint = footprint;
    assisted_teleop::projectFootprint(
      footprint,
      projected_footprint,
      yaw_, speed_, projection_time);
    publisher_->publish(*projected_footprint.get());
  }

  void
  topic_callback(const geometry_msgs::msg::Twist::SharedPtr speed)
  {
    geometry_msgs::msg::Twist speed_var;
    geometry_msgs::msg::PoseStamped out_pose;
    speed_ = speed->linear.x;
    geometry_msgs::msg::TransformStamped tStd = buffer->lookupTransform(
      "map", "base_link",
      builtin_interfaces::msg::Time());
    yaw_ = assisted_teleop::projectPose(
      *buffer,
      global_pose_,
      out_pose,
      speed, projection_time, yaw_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_f_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource timesource;
  timesource.attachClock(clock);
  rclcpp::shutdown();
  return 0;
}
