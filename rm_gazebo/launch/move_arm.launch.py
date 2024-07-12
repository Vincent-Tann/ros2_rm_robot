import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'rm_gazebo'
    # pkg_share = FindPackageShare(package=package_name).find(package_name)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'rm_group_controller'],
        output='screen'
    )

    close_evt = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_launch,
            on_exit=[load_joint_state_controller, load_joint_trajectory_controller],
        )
    )

    move_arm_node = Node(
        package=package_name,
        executable='move_arm',
        name='move_arm',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        close_evt,
        move_arm_node
    ])