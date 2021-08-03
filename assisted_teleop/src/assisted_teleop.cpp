#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_utils.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("pose_listener")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}


#include "turtlesim/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

std::string turtle_name;
//Node handle
rclcpp::Node::SharedPtr node_handle = nullptr;

void pose_callback(const turtlesim::msg::Pose::SharedPtr pose)
{
    //tf broadcaster
    static tf2_ros::TransformBroadcaster pose_broadcaster_(node_handle);
    //Broadcaster message type instantiation
    geometry_msgs::msg::TransformStamped pose_tf;
    //According to the current pose of the turtle, publish the transformation of the world coordinate system
    pose_tf.header.stamp = node_handle->now();
    pose_tf.header.frame_id = "world";
    pose_tf.child_frame_id = turtle_name;
    pose_tf.transform.translation.x = pose->x;
    pose_tf.transform.translation.y = pose->y;
    pose_tf.transform.translation.z = 0.0;
    //yaw to quaternion
    tf2::Quaternion quaternion;
    // yaw, pitch and roll are rotations in z, y, x respectively
    quaternion.setRPY(0,0,pose->theta);
    pose_tf.transform.rotation = tf2::toMsg(quaternion);
    //Post coordinate transformation
    pose_broadcaster_.sendTransform(pose_tf);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
    //Initialize the node
    node_handle = rclcpp::Node::make_shared("turtle_tf_broadcaster");
    //Get the name of the little turtle
    if (argc != 2)
    {
        RCLCPP_ERROR(node_handle->get_logger(), "error Exiting");
        return -1;
    };
    turtle_name = argv[1];
    //Receive the pose information of the little turtle
    auto subscription = node_handle->create_subscription<turtlesim::msg::Pose>(turtle_name+"/pose", 10, pose_callback); 
    //Start receiving topic
    rclcpp::spin(node_handle);
    rclcpp::shutdown();
    return 0;
}
