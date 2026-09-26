import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

        self.cx = 160
        self.cy = 120

        self.subscription = self.create_subscription(
            String, 'target_position', self.position_callback, 10
        )

    def position_callback(self, msg):
        try:
            pos = eval(msg.data)
            x, y, z = pos[0], pos[1], pos[2]
            self.cx = int((x - 0.4) / 0.2 * 80 + 120)
            self.cy = int((y + 0.2) / 0.4 * -80 + 160)
            self.get_logger().info(f'Ball position {pos} -> pixel ({self.cx}, {self.cy})')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse position: {e}')

    def timer_callback(self):
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(img, (self.cx, self.cy), 30, (0, 0, 255), -1)
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
