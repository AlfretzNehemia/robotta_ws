import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory(
                'robotta_bringup')
            + '/config/mapper_params_offline.yaml'
        ],
    )

    # rplidar = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('robotta_bringup'),'launch','rplidar.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d',
            get_package_share_directory('robotta_bringup')
            + '/config/slam_view.rviz'],
    )

    ld = LaunchDescription()
    ld.add_action(slam_node)
    # ld.add_action(rplidar)
    ld.add_action(rviz2_node)

    return ld