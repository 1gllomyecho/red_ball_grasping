import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import ast

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

        # ===== 相机内参（假图 320x240）=====
        self.fx = 50.0
        self.fy = 50.0
        self.cx = 160.0
        self.cy = 120.0

        # ===== 果柄的物理坐标（由 robot_driver 发过来）=====
        # 默认值：图像中心附近，避免第一帧报错
        self.stem_world = [0.5, 0.0, 0.35]

        self.subscription = self.create_subscription(
            String, 'target_position', self.position_callback, 10
        )

    def position_callback(self, msg):
        try:
            pos = ast.literal_eval(msg.data)  # 比 eval 安全
            # robot_driver 发的是果柄物理坐标 [x, y, z]
            self.stem_world = [float(pos[0]), float(pos[1]), float(pos[2])]
            self.get_logger().info(f'Stem world position: {self.stem_world}')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse position: {e}')

    def project(self, world_pos):
        """3D 物理坐标 -> 2D 像素坐标（标准相机模型，共原点）"""
        X, Y, Z = world_pos
        if Z <= 1e-6:
            return None
        u = self.fx * (X / Z) + self.cx
        v = self.fy * (Y / Z) + self.cy
        return int(round(u)), int(round(v))

    def timer_callback(self):
        img = np.zeros((240, 320, 3), dtype=np.uint8)

        pixel = self.project(self.stem_world)
        if pixel is not None:
            u, v = pixel
            # 画绿果柄：一个小矩形，宽 4 像素，高 20 像素
            cv2.rectangle(img, (u - 2, v - 10), (u + 2, v + 10), (0, 255, 0), -1)

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
