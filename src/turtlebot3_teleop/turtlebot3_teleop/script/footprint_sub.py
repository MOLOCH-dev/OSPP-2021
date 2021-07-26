import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf2_ros
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PolygonStamped,
            'local_costmap/published_footprint',
            self.listener_callback,
            10)
        self.subscription2 = self.create_subscription(Twist, 'cmd_vel', self.listener_callback2, 10)
        self.subscription3 = self.create_subscription(Pose2D, 'odom', self.listener_callback3, 10)
        self.subscription  # prevent unused variable warning
        self.subscription2
        self.subscription3
        self.time = 1
        
        
        

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % str(len(msg.polygon.points))) #points[1].x
        self.footprint = msg.polygon.points
        
        
        #print(self.footprint)
        
        
    def listener_callback2(self, msg):
        #self.get_logger().info('I heard: "%s"' % str(len(msg.polygon.points))) #points[1].x
        self.velx = msg.linear.x
        self.vely = msg.linear.y
        self.velz = msg.angular.z
        
        #print(self.velx,self.vely,self.velz)
        #print(self.footprint)
    def listener_callback3(self, msg):
        #self.get_logger().info('I heard: "%s"' % str(len(msg.polygon.points))) #points[1].x
        self.orien = tf2.getYaw(msg.pose.pose.orientation)
        print(self.orien)
        

    def forward_project(self):
#self.get_logger().info('I heard: "%s"' % str(len(msg.polygon.points))) #points[1].x
        self.velx = msg.linear.x
        self.vely = msg.linear.y
        self.velz = msg.angular.z
        new_orien = self.orien + (self.velz*self.time)
        for coord in footprint:
	        coord[0] = coord[0] + self.velz*(self.velx*math.cos(new_orien))
	        coord[1] = coord[1] + self.velz*(self.velx*math.cos(new_orien))
        
        #print(self.velx,self.vely,self,velz)
        print(self.footprint)




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #print(minimal_subscriber.listener_callback)
    rclpy.spin(minimal_subscriber)
	
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
