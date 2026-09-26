import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionToArm(Node):
    def __init__(self):
        super().__init__('vision_to_arm')
        self.subscription = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
        self.publisher = self.create_publisher(String, 'joint_command', 10)
        self.bridge = CvBridge()

        self.declare_parameter('red_lower_h', 0)
        self.declare_parameter('red_upper_h', 10)

        h_low = self.get_parameter('red_lower_h').value
        h_high = self.get_parameter('red_upper_h').value
        self.red_lower = np.array([h_low, 120, 70])
        self.red_upper = np.array([h_high, 255, 255])

        self.start_time = self.get_clock().now()
        self.sent_grasp = False

    def image_callback(self, img_msg):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 2.0:
            return

        if self.sent_grasp:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.red_lower, self.red_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                cmd_msg = String()
                cmd_msg.data = "grasp"
                self.publisher.publish(cmd_msg)
                self.sent_grasp = True

def main(args=None):
    rclpy.init(args=args)
    node = VisionToArm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
