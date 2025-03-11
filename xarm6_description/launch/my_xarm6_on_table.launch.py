import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <-- important import!
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    def on_launch(context, *args, **kwargs):
        # Path to your custom xacro that includes xArm + table
        xacro_path = os.path.join(
            get_package_share_directory('xarm6_description'),
            'urdf',
            'my_xarm6_on_table.xacro'
        )

        # Use Command substitution to convert xacro -> URDF string
        robot_desc = Command(['xacro ', xacro_path])

        # Mark it as "this is a string" not YAML
        robot_description_param = {
            'robot_description': ParameterValue(robot_desc, value_type=str)
        }

        # Robot State Publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description_param]
        )

        # Load an RViz config from xarm6_description or your own
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

        return [rsp_node, rviz_node]

    return LaunchDescription([
        OpaqueFunction(function=on_launch)
    ])
