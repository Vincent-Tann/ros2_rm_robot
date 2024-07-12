import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import xacro

from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    # 启动gazebo
    print(os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'))
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    ld = LaunchDescription([
        # SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'DEBUG'),
        gazebo
    ])

    return ld


# import launch
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     # 获取Gazebo的安装路径
#     gazebo_ros_path = os.path.join(os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], 'share', 'gazebo_ros')
#     gazebo_launch_file = os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')

#     return LaunchDescription([
#         # 包含Gazebo的启动文件
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(gazebo_launch_file)
#         ),

#         # 这里可以添加更多的节点或动作，例如加载机器人模型等
#     ])

# if __name__ == '__main__':
#     generate_launch_description()
