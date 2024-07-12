#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

class MoveArm(Node):

    def __init__(self):
        super().__init__('move_arm')
        self.logger = get_logger('move_arm')

        self.moveit_py = MoveItPy(node_name='moveit_py')
        self.planning_component = self.moveit_py.get_planning_component('rm_group')

        self.logger.info('MoveItPy instance created')
        self.move_arm_to_pose()

    def move_arm_to_pose(self):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'  # Adjust this frame according to your robot
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.4
        pose_goal.pose.position.y = 0.0
        pose_goal.pose.position.z = 0.4
        self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link='end_effector_link')  # Adjust end-effector link name

        self.plan_and_execute()

    def plan_and_execute(self):
        self.logger.info('Planning trajectory')
        plan_result = self.planning_component.plan()

        if plan_result:
            self.logger.info('Executing plan')
            robot_trajectory = plan_result.trajectory
            self.moveit_py.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error('Planning failed')


def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()