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

        self.green_lower = np.array([40, 100, 100])
        self.green_upper = np.array([80, 255, 255])

        self.start_time = self.get_clock().now()
        self.last_sent_time = None
        self.cooldown = 3.0   # 秒。两次发送之间至少隔 3 秒，等 robot_driver 抓完

    def image_callback(self, img_msg):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 2.0:
            return

        now = self.get_clock().now()
        if self.last_sent_time is not None:
            since_last = (now - self.last_sent_time).nanoseconds / 1e9
            if since_last < self.cooldown:
                return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                u = M["m10"] / M["m00"]
                v = M["m01"] / M["m00"]
                cmd_msg = String()
                cmd_msg.data = f"{u:.2f},{v:.2f}"
                self.publisher.publish(cmd_msg)
                self.get_logger().info(f'Sent pixel coordinates: ({u:.2f}, {v:.2f})')
                self.last_sent_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VisionToArm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
