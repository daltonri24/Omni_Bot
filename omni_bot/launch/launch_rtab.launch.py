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
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    rtabmap_mapping_launch = GroupAction(
        actions=[
            
            SetRemap(src='/odom', dst='omni_wheel_controller/odom'),
            SetRemap(src='/rtabmap/map',dst='/map'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')]),
                launch_arguments={
                    'rtabmap_args': '--delete_db_on_start  --Grid/RangeMax 2.5  --Grid/NoiseFilteringMinNeighbors 10  --Grid/MaxObstacleHeight 2.5  --Mem/SaveDepth16Format true',
                    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                    'rgb_topic': '/camera/camera/color/image_raw',
                    'camera_info_topic': '/camera/camera/color/camera_info',
                    'approx_sync': 'false',
                    'frame_id': 'base_link',
                    'rviz': 'false',
                    'rtabmap_viz': 'true',
                    'odom_frame_id': 'odom',
                    'cloud_noise_filtering_radius': '0.05',
                    "cloud_noise_filtering_min_neighbors": '5',
                    'use_sim_time': use_sim_time,
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
                    'rtabmap_viz': 'true',
                    'odom_frame_id': 'odom',
                    'cloud_noise_filtering_radius': '0.05',
                    "cloud_noise_filtering_min_neighbors": '5',
                    'use_sim_time': use_sim_time,
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
        DeclareLaunchArgument(
                    'use_sim_time',
                    default_value='false',
                    description='Use sim time if true'),

        rtabmap_mapping_launch
    ])

