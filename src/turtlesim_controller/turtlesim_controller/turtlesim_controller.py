import rclpy
import rclpy.node
import logging
import random

from rclpy.node import Node
from rclpy.action import ActionClient
from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.action import RotateAbsolute

import math

class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        self.state: str = 'forward'

        self.goal_future: Future = None

        self.x: float = 0
        self.y: float = 0
        self.theta: float = 0
        self.result_response: float = 0
        self.action_working: bool = False

        self.declare_parameter('stop', 'false')

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.logger = self.get_logger()

        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_listener, 10)
        self.action_client = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')

    def send_goal(self, grade):
        goal = RotateAbsolute.Goal()
        goal.theta = math.radians(grade)

        self.action_client.wait_for_server()

        self.goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response)

        

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.result = future.result().result
        self.get_logger().info('Result: {0}'.format(self.result.delta))
        self.result_response = self.result.delta
        # rclpy.shutdown()

    def feedback_callback(self, msg):
        self.feedback = msg.feedback
        self.logger.info(f"feedback: {self.feedback}")

    def timer_callback(self):
        stop_param = self.get_parameter('stop').get_parameter_value().string_value
        degree: int = random.randint(1, 360)

        if (stop_param == 'false'):
            pub_msg = Twist()
            if self.x >= 11 or self.x <= 0 or self.y >= 11 or self.y <= 0:
                self.state = 'backward'
            elif self.state == 'backward':
                self.state = 'rotate'
            else:
                self.state == 'forward'

            ################ State Machine #####################
            if self.state == 'backward':
                pub_msg.linear.x = -2.0
                self.publisher.publish(pub_msg)
                self.logger.info(f"wall: x: {pub_msg}")
            if self.state == 'forward':
                pub_msg.linear.x = 1.0
                self.publisher.publish(pub_msg)
                self.logger.info(f"x: {pub_msg}")
            if self.state == 'rotate':
                if self.result_response == 0 and not self.action_working:
                    self.send_goal(degree)
                    self.action_working = True
                if abs(self.result_response) > 0:
                    self.result_response = 0
                    self.action_working = False
                    self.state = 'forward'
                

    def pose_listener(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        


def main():
    rclpy.init()
    turtlesim_controller = TurtlesimController()
    rclpy.spin(turtlesim_controller)
    turtlesim_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()