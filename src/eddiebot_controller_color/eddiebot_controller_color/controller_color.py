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
ROUNDING_WALK = 4
REC_THR = 50

class Controller_color(Node):
    def __init__(self):
        super().__init__('controller_color')

        self.CALLBACK_TIME = 0.01
        self.ROTATION_TIME = 0.3
        self.ITERATIONS_OF_ROTATION_TIME = self.ROTATION_TIME / self.CALLBACK_TIME
        self.walk_state = IDLE_WALK
        self.last_walk_state = IDLE_WALK

        self.image_sub = self.create_subscription(Image, '/kinect_rgbd_camera/image', self.image_sub_callback, 10)
        self.move_publisher = self.create_publisher(Twist, '/model/eddiebot/cmd_vel', 10)
        self.moving_timer = self.create_timer(self.CALLBACK_TIME, self.moving_timer_callback)

        self.img_wall_and_bnw = None
        self.rounding = False
        self.over = False
        self.init = False
        
        self.more_rotate = 0                                                       # must rotate until 2.5 seconds
        
        self.bridge_for_cam = CvBridge()

    def moving_timer_callback(self):
        tw = Twist()
        # self.get_logger().info(f'self.more_rotate={self.more_rotate}')
        # if self.over:
            # tw.angular.x = 1.0
        # else:
        if self.init:
            if self.walk_state == IDLE_WALK:
                # if self.last_walk_state == YELLOW_WALK and self.more_rotate < self.ITERATIONS_OF_ROTATION_TIME:
                if self.last_walk_state == YELLOW_WALK and self.near_hit(self.img_wall_and_bnw, 50):
                    # self.more_rotate += 1
                    tw.angular.z = -0.5
                elif not self.near_hit(self.img_wall_and_bnw, 50):
                    # self.get_logger().info('HERE')
                    self.more_rotate = 0
                    self.last_walk_state = IDLE_WALK

                if self.last_walk_state == IDLE_WALK:
                    tw.linear.x = 0.5


            if self.walk_state == RED_WALK:
                tw.angular.z= 0.5
            if self.walk_state == YELLOW_WALK:
                self.get_logger().info('HERE')
                tw.angular.z = -0.5
            if self.walk_state == GREEN_WALK:
                tw.angular.z = 0.5
                self.over = True


            self.move_publisher.publish(tw)

        # self.get_logger().info(f"published {tw}")
        # self.get_logger().info("published")
        

    def near_hit(self, img, th):
        total_pixels = img.size
        non_black_pixels = cv2.countNonZero(img)
        percentage = (non_black_pixels / total_pixels) * 100

        if percentage >= th:
            return True
    
        return False

    def split_color(self, img, lower_bound, upper_bound):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array(lower_bound, dtype="uint8")
        upper = np.array(upper_bound, dtype="uint8")

        mask = cv2.inRange(img_hsv, lower, upper)

        img_and = cv2.bitwise_and(img, img, mask=mask)        

        return img_and
    
    def get_img_score(self, img):
        center = int(img.shape[1]/2)
        end = img.shape[1]
        img_yellow_score = np.sum(img[:, center:end] - img[:, 0:center])

        return img_yellow_score


    def image_sub_callback(self, msg):
        self.init = True

        cv_image_cam = self.bridge_for_cam.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        img_yellow_and = self.split_color(img=cv_image_cam, lower_bound=[22, 93, 0], upper_bound=[45, 255, 255])
        img_red_and = self.split_color(img=cv_image_cam, lower_bound=[0, 100, 100], upper_bound=[30, 255, 255])
        img_green_and = self.split_color(img=cv_image_cam, lower_bound=[50, 100, 100], upper_bound=[70, 255, 255])
        img_wall_and = self.split_color(img=cv_image_cam, lower_bound=[80, 100, 100], upper_bound=[100, 255, 255])

        img_yellow_and_bnw = cv2.cvtColor(img_yellow_and, cv2.COLOR_BGR2GRAY)
        img_red_and_bnw = cv2.cvtColor(img_red_and, cv2.COLOR_BGR2GRAY)
        img_green_and_bnw = cv2.cvtColor(img_green_and, cv2.COLOR_BGR2GRAY)

        self.img_wall_and_bnw =  cv2.cvtColor(img_wall_and, cv2.COLOR_BGR2GRAY)

        img_yellow_score = self.get_img_score(img_yellow_and_bnw)

        cv2.imwrite('/home/majiddrn/tmp/cyn.png', self.img_wall_and_bnw)

        tw = Twist()
        if img_yellow_score > 0:
            
            tw.angular.z = -0.5
        else:
            tw.angular.z = 0.5

        if self.near_hit(img_yellow_and_bnw, 95):
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
                # self.last_walk_state = self.walk_state
                # if not self.near_hit(img_wall_and_bnw, 50):
                self.walk_state = IDLE_WALK
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