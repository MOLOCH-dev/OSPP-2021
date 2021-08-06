#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "local_costmap/published_footprint", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
//for (unsigned int i = 0; i < footprint.size() - 1; ++i)
  private:
    void topic_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr footprint) const
    {
    for (unsigned int i = 0; i < footprint->polygon.points.size() - 1; ++i){
      std::cout << " X : " << footprint->polygon.points[i].x << " y : " << footprint->polygon.points[i].y << " z : " << footprint->polygon.points[i].z << " : for i: " << i << std::endl;
      }
    }
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;
   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
