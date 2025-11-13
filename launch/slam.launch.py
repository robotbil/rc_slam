import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context):
    use_sim = LaunchConfiguration('simulation').perform(context)
    simulation = use_sim.lower() == 'true'
    
    # --- Pakke-stier ---
    nav2_package_path = get_package_share_directory('nav2_bringup')
    slam_package_path = get_package_share_directory('slam_toolbox')
    controller_package_path = get_package_share_directory('controller')
    peripheral_package_path = get_package_share_directory('rc_peripherals')
    config_path = os.path.join(get_package_share_directory('rc_slam'), 'config')
    
    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(peripheral_package_path, 'launch/imu_filter.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('simulation'),
        }.items()
    )
    ekf_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(peripheral_package_path, 'launch/odom_to_base_link_ekf.launch.py')
        ]),
        launch_arguments={
            'simulation': LaunchConfiguration('simulation'),
        }.items()
    )


    # --- SLAM Toolbox ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_package_path, 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(config_path, 'mapper_params_online_async.yaml'),
            'use_sim_time': LaunchConfiguration('simulation'),
        }.items()
    )

    # --- Nav2 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_package_path, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(config_path, 'nav2_params.yaml'),
            'use_sim_time': LaunchConfiguration('simulation'),
        }.items()
    )
    
    if not simulation:    
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
        return [
            controller,
            lidar,
            imu_filter_launch,
            ekf_filter_launch,
            TimerAction(period=5.0, actions=[slam]),
            TimerAction(period=0.0, actions=[nav2])
        ]
    else:
        gazebo_backage_path = get_package_share_directory('rc_gazebo')

        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(gazebo_backage_path, 'launch', 'simulator.launch.py')
            ])
        )

        return [
            gazebo_launch,
            imu_filter_launch,
            ekf_filter_launch,
            TimerAction(period=3.0, actions=[slam]),
            TimerAction(period=7.0, actions=[nav2])
        ]

          


def generate_launch_description():
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Set to true when running in simulation'
    )

    return LaunchDescription([
        simulation_arg,
        OpaqueFunction(function=launch_setup)
    ])
