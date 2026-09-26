import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ResultMonitor(Node):
    def __init__(self):
        super().__init__('result_monitor')
        self.subscription = self.create_subscription(
            String, 'grasp_result', self.callback, 10
        )

    def callback(self, msg):
        self.get_logger().info('抓取结果: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ResultMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
