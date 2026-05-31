from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('my_robot_description'), 'urdf', 'my_robot.urdf.xacro']
    )
    world_path = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'), 'worlds', 'wall_world.sdf']
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'), 'config', 'bridge.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('my_robot_bringup'), 'config', 'my_robot.rviz']
    )


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}],
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'my_robot',
                '-z', '0.1',
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

        Node(# add the robot nav node 
            package='my_robot_nav',
            executable='obstacle_nav',
            name='my_robot_nav',
            output='screen'
        ),
        
        Node(# add the robot velocity controller  
            package='my_robot_controll',
            executable='velocity_controller_node',
            name='velocity_controller_node',
            output='screen'
        ),
    ])
