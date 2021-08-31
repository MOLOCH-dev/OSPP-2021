// Copyright 2021 Anushree Sabnis

#include "nav2_assisted_teleop/assisted_teleop.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include <string>
#include <memory>
#ifdef NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
using namespace std::chrono_literals;


namespace nav2_assisted_teleop
{
template<class NodeType>
CostmapSubscriberClass<NodeType>::CostmapSubscriberClass(
  std::shared_ptr<NodeType> node_,
  std::string & topic_name)
: CostmapSubscriber(node_, topic_name) {}

template<class NodeType>
void CostmapSubscriberClass<NodeType>::setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  costmap_msg_ = msg;
  costmap_received_ = true;
}

template<class NodeType>
std::shared_ptr<nav2_costmap_2d::Costmap2D> CostmapSubscriberClass<NodeType>::getCostmap()
{
  this->toCostmap2D();
  if (costmap_ != nullptr) {
    std::cout << "got costmap" << std::endl;
  }
  return costmap_;
}

template CostmapSubscriberClass<rclcpp::Node>::CostmapSubscriberClass(
  std::shared_ptr<rclcpp::Node> node, std::string & topic_name);

template<class NodeType>
FootprintSubscriberClass<NodeType>::FootprintSubscriberClass(
  std::shared_ptr<NodeType> node_,
  std::string & topic_name)
: FootprintSubscriber(node_, topic_name, 10)
{
}

template<class NodeType>
void FootprintSubscriberClass<NodeType>::setFootprint(
  geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::cout << "in set footprint" << std::endl;
  footprint_ = msg;
  footprint_received_ = true;
}

template<class NodeType>
bool FootprintSubscriberClass<NodeType>::getFootprint()
{
  rclcpp::Duration footprint_timeout_(0.1);
  // std::cout << footprint_received_ << "\tfp rec" << std::endl;
  // return this->getFootprint(fp_point_);
  return footprint_received_;
}

template<class NodeType>
void
FootprintSubscriberClass<NodeType>::projectPose(
  double yaw_, double speed_, double angular_vel_, double projection_time,
  geometry_msgs::msg::PoseStamped curr_pose_)
{
  curr_pose_.pose.position.x += projection_time * (speed_ + cos(yaw_));
  curr_pose_.pose.position.y += projection_time * (speed_ + sin(yaw_));
  curr_pose_.pose.orientation.z += projection_time * angular_vel_;
}

template<class NodeType>
void
FootprintSubscriberClass<NodeType>::projectFootprint(
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

template FootprintSubscriberClass<rclcpp::Node>::FootprintSubscriberClass(
  std::shared_ptr<rclcpp::Node> node, std::string & topic_name);

template<class NodeType>
AssistedTeleop<NodeType>::AssistedTeleop(std::shared_ptr<NodeType> node_)
{
  node_int_ = node_;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  while (!tf_buffer_->canTransform(
      global_frame_, base_frame_, tf2::TimePointZero, 1ms,
      &tf_err) && rclcpp::ok())
  {
    updatePose();

  }
  vel_subscriber_ = node_->template create_subscription<geometry_msgs::msg::Twist>(
    vel_topic, 10, std::bind(&AssistedTeleop<NodeType>::vel_callback, this, std::placeholders::_1));

  costmap_sub_ = std::make_shared<CostmapSubscriberClass<NodeType>>(
    node_,
    costmap_topic_);

  footprint_sub_ = std::make_shared<FootprintSubscriberClass<NodeType>>(
    node_,
    footprint_topic_);

  collision_checker_ = std::make_unique<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, *tf_buffer_, node_->get_name(), "map");

  fp_publisher_ = node_->template create_publisher<geometry_msgs::msg::PolygonStamped>(
    fp_pub_topic, 10);


  std::cout << "here" << std::endl;
}

template<class NodeType>
void AssistedTeleop<NodeType>::updatePose()
{
  if (nav2_util::getCurrentPose(
      global_pose_, *tf_buffer_, global_frame_,
      base_frame_,
      transform_timeout))
  {
    yaw_ = tf2::getYaw(global_pose_.pose.orientation);
    std::cout << global_pose_.pose.position.x << "got pose" << std::endl;
  }
}

template<class NodeType>
void AssistedTeleop<NodeType>::vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_ = msg->linear.x;
  angular_vel_ = msg->angular.z;
  // std::cout << "velocity :\t" << msg->linear.x << std::endl;
  rclcpp::Duration footprint_timeout_(0.1);
  costmap_ros_ = costmap_sub_->getCostmap();
  if (speed_ != 0) {
    projection_time = costmap_ros_->getResolution() / speed_;
  }
  // std::cout << "projection_time is : " << projection_time << std::endl;
  updatePose();
  // std::cout << "yaw\t" << yaw_ << std::endl;
  footprint_sub_->projectPose(yaw_, speed_, angular_vel_, projection_time, global_pose_);
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = global_pose_.pose.position.x;
  curr_pose.y = global_pose_.pose.position.y;
  curr_pose.theta = tf2::getYaw(global_pose_.pose.orientation);
  footprint_sub_->setFootprint(projected_footprint_);
  // while (!footprint_sub_->getFootprint()) {
  //   std::cout << collision_checker_->isCollisionFree(curr_pose);
  // }
  
 footprint_sub_->projectFootprint(yaw_, speed_, projection_time);
 fp_publisher_->publish(*projected_footprint_.get());
}
//  explicit initialisation
template AssistedTeleop<rclcpp::Node>::AssistedTeleop(std::shared_ptr<rclcpp::Node> node);
template AssistedTeleop<nav2_util::LifecycleNode>::AssistedTeleop(std::shared_ptr<nav2_util::LifecycleNode> node);
}  // namespace nav2_assisted_teleop


#endif
