import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Topic where joint angles are published
            self.joint_state_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def joint_state_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))  # Map joint names to angles

        self.get_logger().info("Joint Angles:")
        for joint, angle in joint_positions.items():
            self.get_logger().info(f"{joint}: {angle:.4f}")  # Print angles

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
