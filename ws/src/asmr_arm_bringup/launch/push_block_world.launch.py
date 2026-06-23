from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction


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
#


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', arm_urdf_path]), value_type=str)}],
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', push_block_world_path],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        # ExecuteProcess(
        #     cmd=[
        #     "ros2", "run", "tf2_ros", "static_transform_publisher",
        #     "0", "0", "0",
        #     "0", "0", "0",
        #     "map", "odom"
        #     ],
        #     output="screen"
        # ),
        # ExecuteProcess(
        #     cmd=[
        #     "ros2", "run", "tf2_ros", "static_transform_publisher",
        #     "0", "0", "0",
        #     "0", "0", "0",
        #     "odom", "base_link"
        # ],
        # output="screen"
        # ),
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
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     parameters=[{'config_file': bridge_config}],
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])