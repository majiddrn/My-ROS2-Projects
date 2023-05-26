import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

RED_WALK = 0
YELLOW_WALK = 1
GREEN_WALK = 2
IDLE_WALK = 3

class Controller_color(Node):
    def __init__(self):
        super().__init__('controller_color')

        self.CALLBACK_TIME = 0.5
        self.ROTATION_TIME = 2.5
        self.ITERATIONS_OF_ROTATION_TIME = self.ROTATION_TIME / self.CALLBACK_TIME
        self.walk_state = IDLE_WALK

        self.image_sub = self.create_subscription(Image, '/kinect_rgbd_camera/image', self.image_sub_callback, 10)
        self.move_publisher = self.create_publisher(Twist, 'model/eddiebot/cmd_vel', 10)
        self.moving_timer = self.create_timer(self.CALLBACK_TIME, self.moving_timer_callback)

        self.rounding = False
        self.over = False
        
        self.little_more_rotate = 0                                                       # must rotate until 2.5 seconds
        
        self.bridge_for_cam = CvBridge()

    def moving_timer_callback(self):
        if self.little_more_rotate >= self.ITERATIONS_OF_ROTATION_TIME:
            self.little_more_rotate = 0

        if self.rounding or self.little_more_rotate:

            

            if not self.rounding:
                self.little_more_rotate += 1
        

    def split_color(self, img, lower_bound, upper_bound):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array(lower_bound, dtype="uint8")
        upper = np.array(upper_bound, dtype="uint8")

        mask = cv2.inRange(img_hsv, lower, upper)

        img_and = cv2.bitwise_and(img, img, mask=mask)        

        return img_and


    def image_sub_callback(self, msg):
        cv_image_cam = self.bridge_for_cam.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        img_yellow_and = self.split_color(img=cv_image_cam, lower_bound=[22, 93, 0], upper_bound=[45, 255, 255])
        img_red_and = self.split_color(img=cv_image_cam, lower_bound=[0, 100, 100], upper_bound=[30, 255, 255])
        img_green_and = self.split_color(img=cv_image_cam, lower_bound=[50, 100, 100], upper_bound=[70, 255, 255])

        img_yellow_and_bnw = cv2.cvtColor(img_yellow_and, cv2.COLOR_BGR2GRAY)
        img_red_and_bnw = cv2.cvtColor(img_red_and, cv2.COLOR_BGR2GRAY)
        img_green_and_bnw = cv2.cvtColor(img_green_and, cv2.COLOR_BGR2GRAY)

        if np.any(img_yellow_and_bnw > 0):
            self.rounding = True
            self.walk_state = YELLOW_WALK
        
        if np.any(img_red_and_bnw > 0):
            self.rounding = True
            self.walk_state = RED_WALK

        if np.any(img_green_and_bnw > 0):
            self.over = True
        # self.get_logger().info(img_red_and)

        # self.get_logger().info(f'data is {msg.data[20]} and the encoding is {msg.encoding} and msg.data.shape.len={len(msg.data)} and img_yellow_and[0][10]={img_yellow_and[0, 10]}')

def main():
    rclpy.init()

    controller_color = Controller_color()

    rclpy.spin(controller_color)

    controller_color.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()