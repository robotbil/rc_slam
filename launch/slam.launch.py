import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context):
    # --- Pakke-stier ---
    nav2_package_path = get_package_share_directory('nav2_bringup')
    slam_package_path = get_package_share_directory('slam_toolbox')
    controller_package_path = get_package_share_directory('controller')
    peripheral_package_path = get_package_share_directory('rc_peripherals')
    config_path = os.path.join(get_package_share_directory('rc_slam'), 'config')

    # --- Controller ---
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(controller_package_path, 'launch', 'controller.launch.py')
        ])
    )

    # --- Lidar ---
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(peripheral_package_path, 'launch', 'lidar.launch.py')
        ])
    )

    # --- SLAM Toolbox ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_package_path, 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(config_path, 'mapper_params_online_async.yaml')
        }.items()
    )

    # --- Nav2 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_package_path, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(config_path, 'nav2_params.yaml')
        }.items()
    )

    return [
        controller,
        lidar,
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=10.0, actions=[nav2])
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
