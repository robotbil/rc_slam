import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction

from nav2_common.launch import ReplaceString

def launch_setup(context):
    nav2_package_path = get_package_share_directory('nav2_bringup')
    slam_package_path = get_package_share_directory('slam_toolbox')
    controller_package_path = get_package_share_directory('controller')
    peripheral_package_path = get_package_share_directory('rc_peripherals')

    config_path = os.path.join(get_package_share_directory('rc_slam'), 'config')


    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav2_package_path, 'launch/navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(config_path, 'nav2_params.yaml')
        }.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(slam_package_path, 'launch/online_async_launch.py')
        ])
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(controller_package_path, 'launch/controller.launch.py')
        ])
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(peripheral_package_path, 'launch/lidar.launch.py')
        ])
    )


    return [
        controller,
        lidar,
        nav2,
        slam,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
