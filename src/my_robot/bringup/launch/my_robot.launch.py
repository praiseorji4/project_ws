from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
import os
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, ExecuteProcess

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Include rsp.launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('my_robot'), 'bringup', 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_controllers = os.path.join(get_package_share_directory('my_robot'), 'bringup', 'config', 'my_robot_controllers.yaml')
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])


    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    robot_controllers]
    )

    # TimerAction to delay the controller manager
    delayed_controller_manager = TimerAction(period=5.0, actions=[control_node])

    # Spawner nodes
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    )

    # Delayed spawner handlers
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])


# def generate_launch_description():
#     # Arguments

#     # Include rsp.launch.py
#     rsp = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('my_robot'), 'bringup', 'launch', 'rsp.launch.py'
#         )]),
#         launch_arguments={'use_sim_time': "no", 'use_ros2_control': "yes"}.items()
#     )

#     # Paths
#     robot_controllers = os.path.join(get_package_share_directory('my_robot'), 'bringup', 'config', 'my_robot_controllers.yaml')
#     rviz_config_file = os.path.join(get_package_share_directory('my_robot'), 'description', 'rviz', 'my_robot.rviz')

#     # Fetch robot description
#     robot_description = Command(['ros2 param get --hide-typ /robot_state_publisher_2 robot_description'])

#     # Controller Manager Node
#     control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[{'robot_description': robot_description}, robot_controllers],
#         output='both',
#     )

#     # RViz Node
#     # rviz_node = Node(
#     #     package='rviz2',
#     #     executable='rviz2',
#     #     name='rviz2',
#     #     output='log',
#     #     arguments=['-d', rviz_config_file],
#     # )

#     # Delayed Controller Manager Node
#     delayed_controller_manager = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=control_node,
#             on_start=[
#                 LogInfo(msg='Controller Manager node is starting...'),
#                 ExecuteProcess(cmd=['sleep', '5']),  # Adjust as needed
#                 LogInfo(msg='Controller Manager node is now running.'),
#             ],
#         )
#     )

#     # Spawner Nodes
#     diff_drive_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['diff_cont', '--controller-manager', '/controller_manager'],
#         output='both',
#     )

#     joint_broad_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_broad', '--controller-manager', '/controller_manager'],
#         output='both',
#     )

#     # Delayed Spawner Nodes
#     delayed_diff_drive_spawner = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=control_node,
#             on_start=[
#                 LogInfo(msg='Starting diff_drive_spawner...'),
#                 diff_drive_spawner,
#             ],
#         )
#     )

#     delayed_joint_broad_spawner = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=control_node,
#             on_start=[
#                 LogInfo(msg='Starting joint_broad_spawner...'),
#                 joint_broad_spawner,
#             ],
#         )
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
#         DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control if true'),
#         rsp,
#         delayed_controller_manager,
#         delayed_diff_drive_spawner,
#         delayed_joint_broad_spawner,
#     ])

