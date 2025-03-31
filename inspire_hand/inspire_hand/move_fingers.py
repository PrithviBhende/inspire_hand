import rclpy
import random
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RandomFingerMover(Node):
    def __init__(self):
        super().__init__('random_finger_mover')
        
        # Publisher to the joint trajectory controller
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Timer to publish every 5 seconds
        self.timer = self.create_timer(5.0, self.send_random_command)

        # Joint limits (Example values; update with actual URDF constraints)
        self.joint_limits = {
            "thumb_proximal_yaw_joint": (0, 1.308),
            "thumb_proximal_pitch_joint": (0, 0.6),
            "index_proximal_joint": (0, 1.47),
            "middle_proximal_joint": (0, 1.47),
            "ring_proximal_joint": (0, 1.47),
            "pinky_proximal_joint": (0, 1.47),
        }

    def send_random_command(self):
        """Sends random joint positions within limits every 5 seconds."""
        msg = JointTrajectory()
        msg.joint_names = list(self.joint_limits.keys())

        point = JointTrajectoryPoint()
        
        # Generate random joint positions within limits
        point.positions = [
            random.uniform(*self.joint_limits[joint]) for joint in msg.joint_names
        ]
        point.time_from_start.sec = 2  # Smooth movement over 2 seconds

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(f"Moving joints to: {dict(zip(msg.joint_names, point.positions))}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomFingerMover()
    
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info("Stopping hand control...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
