import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        
        if(self.i % 2 == 0):
            msg.linear.x = 3.0
        else:
            msg.linear.x = -3.0
        
        self.publisher_.publish(msg)
        self.i += 1


def main():
    rclpy.init()

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
