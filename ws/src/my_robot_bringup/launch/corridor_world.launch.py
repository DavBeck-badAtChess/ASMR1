import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('my_robot_description'), 'urdf', 'my_robot_arm.urdf.xacro']
    )
    world_path = PathJoinSubstitution(
        [FindPackageShare('my_robot_description'), 'worlds', 'corridor_world.sdf']
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'), 'config', 'bridge.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'), 'config', 'my_robot.rviz']
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': [world_path, ' -r']}.items(),
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command([
                'xacro ', urdf_path,
                ' use_sim_control:=true',
            ]), value_type=str)}],
        ),
        
        gz_sim,

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'my_robot',
                '-z', '0.1',
                '-x', '-0.5',
                '-y', '0',
                '-Y', '0.26',
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),

        Node(
            package='asmr_arm_control',
            executable='kinematics_server',
        ),

        Node(
            package='asmr_arm_control',
            executable='trajectory_server',
        ),
    ])
