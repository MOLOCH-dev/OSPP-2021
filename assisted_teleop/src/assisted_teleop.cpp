#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/geometry_utils.hpp"
#include <rclcpp/time_source.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


class MinimalSubscriber : public rclcpp::Node
{
	// Initialising variables
	geometry_msgs::msg::PoseStamped out_pose_2; //For pose projection
	geometry_msgs::msg::PolygonStamped::SharedPtr projected_footprint; //For footprint projection
	geometry_msgs::msg::Twist speed_var_2;
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
    // Subscribing to /cmd_vel
		subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

		//Publishing to /local_costmap/projected_footprint
		publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("local_costmap/published_footprint", 10);
		timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalSubscriber::pub_callback, this));
		subscription_f_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
		"local_costmap/published_footprint", 5, std::bind(&MinimalSubscriber::footprint_callback, this, _1));


		tf2::Transform transform_; //Declare new transform object
		transform_.setIdentity(); 
		rclcpp::WallRate loop_rate(500);
  		
   }
   
   
   
   
   
  geometry_msgs::msg::PoseStamped transformPose(geometry_msgs::msg::TransformStamped & in_pose, double projection_time,  geometry_msgs::msg::Twist & speed) const //made this const to get rid of [-fpermissive] flag

  {
  	//Forward projecting pose in time
  	geometry_msgs::msg::PoseStamped out_pose;
  	double yaw_ = tf2::getYaw(in_pose.transform.rotation);
  	out_pose.pose.position.x = in_pose.transform.translation.x + projection_time * (speed.linear.x + cos(yaw_));
  	out_pose.pose.position.y = in_pose.transform.translation.y + projection_time * (speed.linear.x + cos(yaw_));
  	out_pose.pose.orientation.z = in_pose.transform.rotation.z + projection_time * (speed.angular.z);
  	out_pose.pose.orientation.z = tf2::getYaw(out_pose.pose.orientation);
  	out_pose.pose.position.z = 0.01;
  	
  	return out_pose;
   
}
		
	
  
  
  private:
  		
  		//tf2 buffer
  		std::unique_ptr<tf2_ros::Buffer> buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  		//Using buffer as transformlistener
  		std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer);
  		
  		
  		geometry_msgs::msg::PoseStamped out_pose;
	
		
  		
		void pub_callback()
		{
			
			
			geometry_msgs::msg::PolygonStamped message;
			
			message = *projected_footprint.get(); //dereferencing SharedPtr to get projected footprint
			
			publisher_->publish(message);
			
		}
  	
  		
  		
  	
  	void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr footprint) const
  	{
  		
  		(const_cast <MinimalSubscriber*> (this) )->projected_footprint = footprint; //had to const cast projected footprint 
  		double projection_time = 1.5;
  		
  		// Forward projecting all points of the footprint
  		for (unsigned int i = 0; i < footprint->polygon.points.size(); ++i){
      	projected_footprint->polygon.points[i].x = footprint->polygon.points[i].x + projection_time *( speed_var_2.linear.x *(cos(out_pose_2.pose.orientation.z))) ;
      	projected_footprint->polygon.points[i].y = footprint->polygon.points[i].y + projection_time *( speed_var_2.linear.x *(sin(out_pose_2.pose.orientation.z))) ;
      	std::cout << "x : " << projected_footprint->polygon.points[i].x << std::endl;
      	std::cout << "y : " << projected_footprint->polygon.points[i].y << std::endl;
      	std::cout << "z : " << projected_footprint->polygon.points[i].z << std::endl;
      }
      
  		
  	}

  		
  	
  
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr speed) const
    {
      
      
  		geometry_msgs::msg::Twist speed_var;
  		double projection_time = 1.5;
  		geometry_msgs::msg::PoseStamped out_pose;
  		
  		
      speed_var.linear.x = speed->linear.x;
      speed_var.linear.y = speed->linear.y;
      speed_var.linear.z = speed->linear.z;
      speed_var.angular.x = speed->angular.x;
      speed_var.angular.y = speed->angular.y;
      speed_var.angular.z = speed->angular.z;
      (const_cast <MinimalSubscriber*> (this) )->speed_var_2.linear.x = speed->linear.x;
      (const_cast <MinimalSubscriber*> (this) )->speed_var_2.angular.z = speed->angular.z;
      
      //Transforming map to odom to get pose
      geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform("map", "odom", builtin_interfaces::msg::Time());
     
      (const_cast <MinimalSubscriber*> (this) )->out_pose_2 = transformPose(transformStamped, projection_time, speed_var);
      
			
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


