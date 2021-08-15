#ifndef ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
#define ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_

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

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace assisted_teleop
{

double
projectPose(
	tf2_ros::Buffer & tf_buffer,
	geometry_msgs::msg::PoseStamped global_pose_,
	geometry_msgs::msg::PoseStamped & out_pose, 
    const geometry_msgs::msg::Twist::SharedPtr speed,
    double projection_time, double yaw_);


void
projectFootprint(
	const geometry_msgs::msg::PolygonStamped::SharedPtr footprint_,
	geometry_msgs::msg::PolygonStamped::SharedPtr projected_footprint_,
	double yaw_, double speed_, double projection_time );

}

#endif