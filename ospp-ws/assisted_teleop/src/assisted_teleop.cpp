// Copyright 2021 Anushree Sabnis

#include "assisted_teleop/assisted_teleop.hpp"

namespace assisted_teleop
{


const double transform_timeout = 2.0;

double
projectPose(
  tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::PoseStamped global_pose_,
  geometry_msgs::msg::PoseStamped & out_pose,
  const geometry_msgs::msg::Twist::SharedPtr speed,
  double projection_time, double yaw_)
{
  if (nav2_util::getCurrentPose(
      global_pose_, tf_buffer, "map",
      "base_link",
      transform_timeout))
  {
    out_pose.pose.orientation.z = global_pose_.pose.orientation.z;
    yaw_ = tf2::getYaw(global_pose_.pose.orientation);
    out_pose.pose.position.x = global_pose_.pose.position.x + projection_time *
      (speed->linear.x + cos(yaw_));
    out_pose.pose.position.y = global_pose_.pose.position.y + projection_time *
      (speed->linear.y + sin(yaw_));
    out_pose.pose.position.z = 0.0;
  }
  return yaw_;
}

void
projectFootprint(
  const geometry_msgs::msg::PolygonStamped::SharedPtr footprint_,
  geometry_msgs::msg::PolygonStamped::SharedPtr projected_footprint_,
  double yaw_, double speed_, double projection_time)
{
  projected_footprint_ = footprint_;
  for (unsigned int i = 0; i < footprint_->polygon.points.size(); ++i) {
    projected_footprint_->polygon.points[i].x = footprint_->polygon.points[i].x + projection_time *
      (speed_ * (cos(yaw_)));
    projected_footprint_->polygon.points[i].y = footprint_->polygon.points[i].y + projection_time *
      (speed_ * (sin(yaw_)));
  }
}
}  // namespace assisted_teleop
