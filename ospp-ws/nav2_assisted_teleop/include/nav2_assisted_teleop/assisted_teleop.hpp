// Copyright 2021 Anushree Sabnis

#ifndef NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
#define NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tf2_ros/create_timer_ros.h"
#include <tf2_ros/transform_listener.h> //NOLINT
// #include <tf2/utils.h> //NOLINT
#include "tf2_ros/buffer.h" //NOLINT

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/time_source.hpp> //NOLINT
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "nav2_assisted_teleop/assisted_teleop.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

using std::placeholders::_1;

namespace nav2_assisted_teleop
{
geometry_msgs::msg::PolygonStamped::SharedPtr projected_footprint_;
std::vector<geometry_msgs::msg::Point> fp_point_;


template<class NodeType>
class CostmapSubscriberClass : public nav2_costmap_2d::CostmapSubscriber
{
public:
  CostmapSubscriberClass(
    std::shared_ptr<NodeType> node_,
    std::string & topic_name);
  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg);
  std::shared_ptr<nav2_costmap_2d::Costmap2D> getCostmap();
};

template<class NodeType>
class FootprintSubscriberClass : public nav2_costmap_2d::FootprintSubscriber
{
public:
  FootprintSubscriberClass(
    std::shared_ptr<NodeType> node_,
    std::string & topic_name);
  void setFootprint(geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  bool getFootprint();
  void
  projectFootprint(
    double yaw_, double speed_, double projection_time);
  void projectPose(
    double yaw_, double speed_, double angular_vel_,
    double projection_time, geometry_msgs::msg::PoseStamped curr_pose_);
};

template<class NodeType = rclcpp::Node>
class AssistedTeleop
{
public:
  typedef std::shared_ptr<NodeType> NodePtr;
  NodePtr node_int_;
  void vel_callback(geometry_msgs::msg::Twist::SharedPtr msg);
  void updatePose();
  bool isCollisionImminent();
  AssistedTeleop();
  explicit AssistedTeleop(NodePtr node);
  // :node_(node) {}
  ~AssistedTeleop() = default;
  std::shared_ptr<CostmapSubscriberClass<NodeType>> costmap_sub_;
  std::shared_ptr<FootprintSubscriberClass<NodeType>> footprint_sub_;

protected:
  std::string vel_topic = "cmd_vel";
  std::string footprint_topic_ = "local_costmap/published_footprint";
  double speed_;
  double angular_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr fp_publisher_;
  geometry_msgs::msg::PoseStamped global_pose_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double yaw_;
  const std::string global_frame_ = "odom";
  const std::string base_frame_ = "base_link";
  const double transform_timeout = 2.0;
  double fp_cost_;
  std::string tf_err;
  std::string costmap_topic_ = "local_costmap/costmap_raw";
  std::unique_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ros_;
  double projection_time;
  std::string fp_pub_topic = "local_costmap/projected_footprint";
};
}  // namespace nav2_assisted_teleop
#endif  // NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
