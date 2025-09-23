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
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )



    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
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
        launch_arguments={'gz_args': ['-s -r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
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

    omni_cont = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "omni_wheel_controller",
        ]
    )

    joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
        ]
    )


    rtabmap_mapping_launch = GroupAction(
        actions=[
            
            SetRemap(src='/odom', dst='omni_wheel_controller/odom'),
            SetRemap(src='/rtabmap/map',dst='/map'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')]),
                launch_arguments={
                    'rtabmap_args': '--delete_db_on_start  --Grid/RangeMax 2.5  --Grid/NoiseFilteringMinNeighbors 10  --Grid/MaxObstacleHeight 2.5  --Mem/SaveDepth16Format true',
                    'depth_topic': '/camera/depth/image_raw',
                    'rgb_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info',
                    'approx_sync': 'false',
                    'frame_id': 'base_link',
                    'rviz': 'false',
                    'rtabmap_viz': 'false',
                    'odom_frame_id': 'odom',
                    'cloud_noise_filtering_radius': '0.05',
                    "cloud_noise_filtering_min_neighbors": '5',
                    'use_sim_time': 'true',
                }.items() # May be useful to look into these parameters: Rtabmap/TimeThr or Rtabmap/MemoryThr
            )
        ]
    )


    rtabmap_localization_launch = GroupAction(
        actions=[
            
            SetRemap(src='/odom', dst='omni_wheel_controller/odom'),
            SetRemap(src='/rtabmap/map',dst='/map'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')]),
                launch_arguments={
                    'rtabmap_args': '--Grid/RangeMax 2.5  --Grid/NoiseFilteringMinNeighbors 10  --Grid/MaxObstacleHeight 2.5  --Mem/SaveDepth16Format true',
                    'depth_topic': '/camera/depth/image_raw',
                    'rgb_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info',
                    'approx_sync': 'false',
                    'frame_id': 'base_link',
                    'rviz': 'false',
                    'rtabmap_viz': 'false',
                    'odom_frame_id': 'odom',
                    'cloud_noise_filtering_radius': '0.05',
                    "cloud_noise_filtering_min_neighbors": '5',
                    'use_sim_time': 'true',
                    'localization': 'true',
                    'Mem/IncrementalMemory':'False',
                    'Mem/InitWMWithAllNodes':'True',
                    'Mem/LocalizationDataSaved':'True',
                    'RGBD/OptimizeFromGraphEnd':'False',
                    'RGBD/SavedLocalizationIgnored':'True',
                    'Rtabmap/TimeThr':'0'
                }.items() # May be useful to look into these parameters: Rtabmap/TimeThr or Rtabmap/MemoryThr
            )
        ]
    )




    # Launch them all!
    return LaunchDescription([
        rsp,
        world_arg,
        gazebo,
        ros_gz_bridge,
        ros_gz_image_bridge,
        gz_spawn_entity,

        TimerAction(
            period=10.0,
            #actions=[omni_cont, joint_broad, rtabmap_mapping_launch],
            actions=[omni_cont, joint_broad, rtabmap_localization_launch],
        )
    ])

