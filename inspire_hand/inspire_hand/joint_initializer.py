import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointInitializer(Node):
    def __init__(self):
        super().__init__('joint_initializer')
        self.publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.timer = self.create_timer(2.0, self.set_initial_positions)
        self.get_logger().info("Joint initializer node started.")

    def set_initial_positions(self):
        msg = JointTrajectory()
        msg.joint_names = [
            "thumb_proximal_yaw_joint",
            "thumb_proximal_pitch_joint",
            "index_proximal_joint",
            "middle_proximal_joint",
            "ring_proximal_joint",
            "pinky_proximal_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 1.5708]
        point.time_from_start.sec = 1  # Ensure smooth transition

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info("Published initial joint positions.")

def main(args=None):
    rclpy.init(args=args)
    node = JointInitializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
