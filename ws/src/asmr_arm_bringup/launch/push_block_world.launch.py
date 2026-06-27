import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    arm_urdf_path = PathJoinSubstitution(
        [FindPackageShare('asmr_arm_description'), 'urdf', 'arm.urdf.xacro']
    )

    push_block_world_path = PathJoinSubstitution(
        [FindPackageShare('asmr_arm_description'), 'worlds', 'push_block_world.sdf']
    )
    
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('asmr_arm_bringup'), 'config', 'bridge.yaml']
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('asmr_arm_bringup'), 'config', 'asmr_arm.rviz']
    )


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': [push_block_world_path, ' -r']}.items(),
    )

    return LaunchDescription([
        Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output = "screen",
           parameters=[{'robot_description': ParameterValue(Command(['xacro ', arm_urdf_path, ' use_sim_control:=true']), value_type=str)}],
        ),
        gz_sim,
        


        Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen',
),

        Node(
    package='controller_manager',
    executable='spawner',
    arguments=['arm_pid_controller'],
    output='screen',
),
        
        Node(
            package='asmr_arm_mission',
            executable='push_block_mission',
        ),

        Node(
            package='asmr_arm_control',
            executable='kinematics_server',
        ),

        Node(
            package='asmr_arm_control',
            executable='trajectory_server',
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'asmr_arm',
                '-z', '0.0',
                '-x', '0.0',
                '-y', '0.0',
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config,
                       'use_sim_time:=true'],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}],
        ),
    ])