import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/rm_group_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2, self.publish_trajectory)

    def publish_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Adjust the joint names as necessary

        point = JointTrajectoryPoint()
        point.positions = [1.5, -0.5, 1.0, -0.8, 1.0, 1.0]  # Target positions for each joint
        point.time_from_start.sec = 5  # Set the time to reach the target position

        trajectory.points.append(point)
        self.publisher.publish(trajectory)
        self.get_logger().info('Publishing trajectory')

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = SimpleTrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
