import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    def on_launch(context, *args, **kwargs):
        # Path to Xacro (robot + table)
        xacro_path = os.path.join(
            get_package_share_directory('xarm6_description'),
            'urdf',
            'my_xarm6_on_table.xacro'
        )

        # Convert Xacro → URDF
        robot_desc = Command(['xacro ', xacro_path])
        robot_description_param = {
            'robot_description': ParameterValue(robot_desc, value_type=str)
        }

        # Robot State Publisher (for the URDF’s static transforms)
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param]
        )

        # Add a joint_state_publisher to publish joint positions
        joint_state_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            # Optionally force the initial positions to 0 (or any other defaults)
            parameters=[{
                # set the URDF param again if you like, or just rely on the param in robot_state_publisher
                'use_gui': True
            }]
        )

        # Rviz
        rviz_config = PathJoinSubstitution([
            FindPackageShare('xarm6_description'),
            'rviz',
            'display.rviz'
        ])
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )

        return [rsp_node, joint_state_pub, rviz_node]

    return LaunchDescription([
        OpaqueFunction(function=on_launch)
    ])
