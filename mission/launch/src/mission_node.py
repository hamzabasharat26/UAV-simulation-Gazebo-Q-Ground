import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission')
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(1.0, self.mission)

    def mission(self):
        # Takeoff and move to finish line
        pose = PoseStamped()
        pose.pose.position.x = 10.0  # forward
        pose.pose.position.y = 0.0
        pose.pose.position.z = 5.0  # altitude
        self.pose_pub.publish(pose)
        self.get_logger().info("🚁 Drone moving towards finish line...")

def main(args=None):
    rclpy.init(args=args)
    node = DroneMission()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
