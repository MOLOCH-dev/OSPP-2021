#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/geometry_utils.hpp"
#include <rclcpp/time_source.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

using std::placeholders::_1;
using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::geometry_utils::euclidean_distance;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      

      
      tf2::Transform transform_;
  		transform_.setIdentity();
  		geometry_msgs::msg::PoseStamped message;
  		rclcpp::WallRate loop_rate(500);
  		
  		
   }
   
  geometry_msgs::msg::PoseStamped transformPose(geometry_msgs::msg::TransformStamped & in_pose, double projection_time,  geometry_msgs::msg::Twist & speed) const //made this const to get rid of [-fpermissive] flag

  {
  	geometry_msgs::msg::PoseStamped out_pose;
  	double yaw_ = tf2::getYaw(in_pose.transform.rotation);
  	out_pose.pose.position.x = in_pose.transform.translation.x + projection_time * (speed.linear.x + cos(yaw_));
  	out_pose.pose.position.y = in_pose.transform.translation.y + projection_time * (speed.linear.x + cos(yaw_));
  	out_pose.pose.orientation.z = in_pose.transform.rotation.z + projection_time * (speed.angular.z);
  	out_pose.pose.position.z = 0.01;
  	
  	return out_pose;
   
}


  
  
  private:
  		std::unique_ptr<tf2_ros::Buffer> buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  		std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer);
  		//geometry_msgs::msg::PoseStamped & out_pose;
  		
  		
  		
  		
  	
  
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr speed) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->linear.c_str());
      
  		geometry_msgs::msg::Twist speed_var;
  		double projection_time = 0.1;
  		geometry_msgs::msg::PoseStamped out_pose;
      speed_var.linear.x = speed->linear.x;
      speed_var.linear.y = speed->linear.y;
      speed_var.linear.z = speed->linear.z;
      speed_var.angular.x = speed->angular.x;
      speed_var.angular.y = speed->angular.y;
      speed_var.angular.z = speed->angular.z;
      geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform("map", "base_link", builtin_interfaces::msg::Time());
      //std::cout << speed_var.linear.x << " : map to base_link: " << tf2::getYaw(transformStamped.transform.rotation) << std::endl;
      out_pose = transformPose(transformStamped, projection_time, speed_var);
      
      std::cout << "In pose is : " << transformStamped.transform.translation.x << " , " << transformStamped.transform.translation.y << " , " << transformStamped.transform.rotation.z << "\nOut pose is: " << out_pose.pose.position.x << " , " << out_pose.pose.position.y << " , " << out_pose.pose.orientation.z << std::endl;
			
    }
    
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
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
  //ament_target_dependencies(assisted_teleop_node rclcpp std_msgs geometry_msgs tf2 tf2_ros)
//add_executable(tf2_listener src/tf2_listener.cpp)
}


