import launch
from launch.actions import RegisterEventHandler, LogInfo, SetEnvironmentVariable, IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.substitutions import LocalSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnShutdown

def generate_launch_description():
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    gazebo_launch_path = PathJoinSubstitution([ros_gz_sim_path, 'launch', 'gz_sim.launch.py'])
    return launch.LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([FindPackageShare('trilateration_nodes'), 'models'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args' : PathJoinSubstitution([FindPackageShare('trilateration_nodes'), 'worlds/simworld.sdf']),
                'on_exit_shutdown' : 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['world/trilateration_world/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']
        ),
        Node(
            package='trilateration_nodes',
            executable='rangefinder'
        ),
        Node(
            package='trilateration_nodes',
            executable='trilateration_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                 '-d', 
                 PathJoinSubstitution([
                    FindPackageShare('trilateration_nodes'), 'rviz', 'config.rviz'
                    ])
            ],
        ),
        Node(
            package='trilateration_nodes',
            executable='visualizernode'
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Shutting Down...', LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])


