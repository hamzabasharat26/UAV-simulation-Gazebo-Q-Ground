import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class PerceptionStub(Node):
    def __init__(self):
        super().__init__("perception_stub")
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.cb_image, 10)
        self.last = time.time()

    def cb_image(self, msg):
        now = time.time()
        dt = now - self.last
        self.get_logger().info(f"Got image — dt={dt:.3f}s")
        self.last = now

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionStub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

