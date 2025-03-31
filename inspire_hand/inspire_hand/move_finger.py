import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FingerMover(Node):
    def __init__(self):
        super().__init__('finger_mover')
        
        # Publisher to the joint trajectory controller
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Timer to continuously publish every 0.1 seconds (10Hz)
        self.timer = self.create_timer(0.1, self.send_command)

    def send_command(self):
        """Continuously sends the target positions to maintain the hand's posture."""
        msg = JointTrajectory()
        msg.joint_names = [
            "thumb_intermediate_joint",
            "index_intermediate_joint",
            "middle_intermediate_joint",
            "ring_intermediate_joint",
            "pinky_intermediate_joint"
        ]  # Joints to move

        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.5, 0.5, 0.5, 0.5]  # Hold at zero degrees
        point.time_from_start.sec = 2  # Move smoothly over 2 seconds

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info("Holding hand position.")

def main(args=None):
    rclpy.init(args=args)
    node = FingerMover()
    
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info("Stopping hand control...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
