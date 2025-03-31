import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FingerMover(Node):
    def __init__(self):
        super().__init__('finger_mover')
        
        # Publisher to the joint trajectory controller
        self.publisher = self.create_publisher(JointTrajectory,'/joint_trajectory_controller/joint_trajectory',10)
        self.timer = self.create_timer(2.0, self.send_command)  # Run once after 2s

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = ["index_intermediate_joint"]  # Joint to move

        point = JointTrajectoryPoint()
        point.positions = [0.785]  # 45 degrees in radians
        point.time_from_start.sec = 2  # Move over 2 seconds

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info("Sent command to move index_intermediate_joint by 45 degrees.")
        self.destroy_timer(self.timer)  # Stop timer after publishing

def main(args=None):
    rclpy.init(args=args)
    node = FingerMover()
    rclpy.spin(node)  # Keep the node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
