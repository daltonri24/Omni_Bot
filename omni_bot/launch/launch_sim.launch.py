import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    package_name='omni_bot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'ionic.sdf'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
        ],
    )



    # Launch them all!
    return LaunchDescription([
        world_arg,
        gazebo,
        ros_gz_bridge,
        ros_gz_image_bridge,
        gz_spawn_entity,
        TimerAction(
            period=10.0,
            actions=[rsp]
        )
    ])

