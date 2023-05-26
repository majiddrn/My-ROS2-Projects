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
FOLLOW_WALK = 4
REC_THR = 50

class Controller_color(Node):
    def __init__(self):
        super().__init__('controller_color')

        self.CALLBACK_TIME = 0.001
        self.ROTATION_TIME = 0.3
        self.ITERATIONS_OF_ROTATION_TIME = self.ROTATION_TIME / self.CALLBACK_TIME
        self.walk_state = IDLE_WALK
        self.last_walk_state = IDLE_WALK

        self.image_sub = self.create_subscription(Image, '/kinect_rgbd_camera/image', self.image_sub_callback, 10)
        self.move_publisher = self.create_publisher(Twist, '/model/eddiebot/cmd_vel', 10)
        self.moving_timer = self.create_timer(self.CALLBACK_TIME, self.moving_timer_callback)

        self.img_wall_and_bnw = None
        self.yellow_wall = None
        self.green_wall = None
        self.red_wall = None

        self.rounding = False
        self.over = False
        self.init = False
        
        self.more_rotate = 0                                                       # must rotate until 2.5 seconds
        
        self.bridge_for_cam = CvBridge()

    def moving_timer_callback(self):
        tw = Twist()

        if self.walk_state == IDLE_WALK:
            tw.linear.x = 0.5
        if self.walk_state == YELLOW_WALK:
            tw.angular.z = -0.5
            tw.linear.x = 0.1
        if self.walk_state == RED_WALK:
            tw.angular.z = 0.5
            tw.linear.x = 0.1
        if self.walk_state == FOLLOW_WALK:
            tw.linear.x = 0.2
            score = self.get_img_score(self.img_wall_and_bnw)
            if score < 0:
                self.get_logger().info('go right it"s negetive')
                tw.angular.z = -0.05
            elif score > 0:
                self.get_logger().info('go left it"s positive')
                tw.angular.z = 0.05

        self.move_publisher.publish(tw)
        

    def near_hit(self, img, th):
        total_pixels = img.size
        non_black_pixels = cv2.countNonZero(img)
        percentage = (non_black_pixels / total_pixels) * 100

        if percentage >= th:
            return True
    
        return False
    
    def threshold_image(self, image, threshold):
        _, thresholded = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)

        return thresholded
    
    def set_pixels_to_255(self, image):
        image[image > 0] = 255
        return image


    def split_color(self, img, lower_bound, upper_bound):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array(lower_bound, dtype="uint8")
        upper = np.array(upper_bound, dtype="uint8")

        mask = cv2.inRange(img_hsv, lower, upper)

        img_and = cv2.bitwise_and(img, img, mask=mask)        

        return img_and
    
    def make_image_clean(self, image):
        image = np.where(image > 0, 255, image)
        image = np.where(image < 0, 0, image)
        return image
    
    def get_img_score(self, image):
        height, width = image.shape[:2]

        # Divide the image into left and right halves
        left_half = image[:, :width // 2]
        right_half = image[:, width // 2:]

        # Count the number of white pixels (255) on each half
        left_count = np.count_nonzero(left_half == 255)
        right_count = np.count_nonzero(right_half == 255)

        if left_count > right_count:
            return -1
        elif left_count < right_count:
            return 1
        else:
            return 0


    def image_sub_callback(self, msg):
        self.init = True

        cv_image_cam = self.bridge_for_cam.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image_cam_gray = cv2.cvtColor(cv_image_cam, cv2.COLOR_BGR2GRAY)

        img_yellow_and = self.split_color(img=cv_image_cam, lower_bound=[22, 93, 0], upper_bound=[45, 255, 255])
        img_red_and = self.split_color(img=cv_image_cam, lower_bound=[0, 100, 100], upper_bound=[30, 255, 255])
        img_green_and = self.split_color(img=cv_image_cam, lower_bound=[50, 100, 100], upper_bound=[70, 255, 255])
        img_wall_and = self.split_color(img=cv_image_cam, lower_bound=[80, 100, 100], upper_bound=[100, 255, 255])

        img_yellow_and_bnw = cv2.cvtColor(img_yellow_and, cv2.COLOR_BGR2GRAY)
        img_red_and_bnw = cv2.cvtColor(img_red_and, cv2.COLOR_BGR2GRAY)
        img_green_and_bnw = cv2.cvtColor(img_green_and, cv2.COLOR_BGR2GRAY)

        self.yellow_wall = self.set_pixels_to_255(img_yellow_and_bnw)
        self.red_wall = self.set_pixels_to_255(img_red_and_bnw)
        self.green_wall = self.set_pixels_to_255(img_green_and_bnw)
        self.img_wall_and_bnw = self.threshold_image(cv_image_cam_gray, 65)

        cv2.imwrite('/home/majiddrn/tmp/c.png', self.img_wall_and_bnw)

        if self.near_hit(img_yellow_and_bnw, 80):
            self.rounding = True
            self.last_walk_state = self.walk_state
            self.walk_state = YELLOW_WALK
        elif self.near_hit(img_red_and_bnw, 50):
            self.rounding = True
            self.last_walk_state = self.walk_state
            self.walk_state = RED_WALK
        elif self.near_hit(img_green_and_bnw, 50):
            self.last_walk_state = self.walk_state
            self.over = True
        else:
            if not self.near_hit(img_yellow_and_bnw, 10):
                self.walk_state = FOLLOW_WALK
        

def main():
    rclpy.init()

    controller_color = Controller_color()

    rclpy.spin(controller_color)

    controller_color.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()