import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscribe_laser = self.create_subscription(
            LaserScan,
            "/lidar",
            self.laser_callback,
            10)
        
        self.publish_robot = self.create_publisher(Twist,
            "/cmd_vel",
            10)
    
    def laser_callback(self, msg):
        wall = False
        robot_twist = Twist()

        for i in range(len(msg.ranges)):
            if msg.ranges[i] < 0.5:
                self.get_logger().info(f'{msg.ranges[i]}')
                wall = True
        
        if wall:
            robot_twist.linear.x = -1.0
            robot_twist.angular.z = 2.0
        else:
            robot_twist.linear.x = 0.5
            robot_twist.angular.z = 0.0
    
            

        self.publish_robot.publish(robot_twist)
        
        

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()