# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from moveit.core.robot_state import RobotState
# from moveit.core.robot_model import RobotModel
# # from moveit.core.planning_scene_interface import PlanningSceneInterface
# import geometry_msgs.msg

# class MoveItStateTester(Node):
#     def __init__(self):
#         super().__init__('moveit_state_tester')
#         self.robot_model = RobotModel(
#             urdf_xml_path="/home/txszju/Code/addx/ws_rm_robot/src/ros2_rm_robot/rm_description/urdf/rm_65_gazebo.urdf",
#             srdf_xml_path="/home/txszju/Code/addx/ws_rm_robot/src/ros2_rm_robot/rm_moveit2_config/rm_65_config/config/rm_65_description.srdf")
#         self.robot_state = RobotState(self.robot_model)
#         # self.planning_scene_interface = PlanningSceneInterface(self.robot_model)

#         # 订阅机器人状态主题
#         self.joint_state_sub = self.create_subscription(
#             geometry_msgs.msg.,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#     def joint_state_callback(self, msg):
#         # 这里可以添加代码来处理接收到的关节状态消息
#         print("Received joint states:", msg)

#     def test_robot_state(self):
#         # 获取当前机器人状态
#         current_state = self.robot_state.get_current_state()

#         # 打印当前关节位置
#         joint_positions = current_state.get_joint_group_positions("arm_group")
#         print("Current joint positions:", joint_positions)

#         # 设置机器人到一个新状态
#         new_positions = [0.5, -0.5, 0.5]  # 假设的关节位置
#         self.robot_state.set_joint_group_positions("arm_group", new_positions)

#         # 打印设置后的状态
#         print("New joint positions:", self.robot_state.get_joint_group_positions("arm_group"))

#         # 这里可以添加更多的MoveIt 2 API调用来测试其他功能

#     def setup(self):
#         # 调用test_robot_state来测试
#         self.test_robot_state()

# def main(args=None):
#     rclpy.init(args=args)
#     tester = MoveItStateTester()
#     tester.setup()
#     rclpy.spin(tester)
#     tester.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import RobotModel
from geometry_msgs.msg import Pose
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import xacro

class MoveItTestNode(Node):
    def __init__(self):
        super().__init__('moveit_test_node')
        self.declare_parameter('robot_description', '')
        self.robot_description_param = self.get_parameter('robot_description').value

        # 加载URDF文件
        # urdf_path = os.path.join(get_package_share_directory('rm_gazebo'), 'config', 'complete_scene.urdf.xacro')
        urdf_path = "/home/txszju/Code/addx/ws_rm_robot/src/ros2_rm_robot/rm_gazebo/config/gazebo_65_description.urdf.xacro"
        # doc = xacro.process_file(urdf_path)
        # print(doc)

        doc = xacro.parse(open(urdf_path))
        xacro.process_doc(doc)
        robot_description = doc.toxml()
        # print(robot_description)
        # robot_description = doc.toprettyxml(indent='  ')

        # 初始化RobotModel和RobotState
        self.robot_model = RobotModel(
            # robot_description,
            urdf_xml_path="/home/txszju/Code/addx/ws_rm_robot/src/ros2_rm_robot/rm_gazebo/config/gazebo_65_description.urdf",
            srdf_xml_path="/home/txszju/Code/addx/ws_rm_robot/src/ros2_rm_robot/rm_moveit2_config/rm_65_config/config/rm_65_description.srdf"
            )
        self.robot_state = RobotState(self.robot_model)

        # 测试方法
        self.test_robot_state_methods()

    def test_robot_state_methods(self):
        # 打印机器人模型的根链接名称
        # print("Root link:", self.robot_model.get_root_link_name())
        
        print("robot state info:")
        print(self.robot_state.state_tree)

        # 设置关节组位置
        joint_model_group_name = 'rm_group'  # 根据你的urdf文件中的关节组名称修改
        # positions = np.zeros(self.robot_model.get_joint_model_group(joint_model_group_name).get_variable_count())
        # self.robot_state.set_joint_group_positions(joint_model_group_name, positions)

        # 获取关节组位置
        joint_positions = self.robot_state.get_joint_group_positions(joint_model_group_name)
        print(f"Joint positions for group {joint_model_group_name}:", joint_positions)

        # 获取全局链接变换
        link_name = 'gripper_base_link'  # 根据你的urdf文件中的末端执行器链接名称修改
        global_transform = self.robot_state.get_global_link_transform(link_name)
        print(f"Global transform for link {link_name}:\n", global_transform)

        # 设置关节组速度
        # velocities = np.ones(self.robot_model.get_joint_model_group(joint_model_group_name).get_variable_count())
        # self.robot_state.set_joint_group_velocities(joint_model_group_name, velocities)

        # 获取关节组速度
        joint_velocities = self.robot_state.get_joint_group_velocities(joint_model_group_name)
        print(f"Joint velocities for group {joint_model_group_name}:", joint_velocities)
        
        pose = self.robot_state.get_pose('Link4')
        print("end pose:\n", pose)
        

        # 设置随机位置
        self.robot_state.set_to_random_positions()
        
        # 更新机器人状态
        self.robot_state.update()
        print("Robot state updated")
        
        # 获取关节组位置
        joint_positions = self.robot_state.get_joint_group_positions(joint_model_group_name)
        print(f"Joint positions for group {joint_model_group_name}:", joint_positions)



def main(args=None):
    rclpy.init(args=args)
    node = MoveItTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()