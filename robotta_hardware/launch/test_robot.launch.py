import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('robotta_description'),
        'urdf',
        'robotta.urdf')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

        # urdf_path = os.path.join(get_package_share_directory('robotta_description'), 'urdf', 'robotta.urdf.xacro')
    # urdf_doc = xacro.parse(open(urdf_path, 'r'))
    # xacro.process_doc(urdf_doc)
    # robotta_description = urdf_doc.toxml()

    test_controller = os.path.join(
        get_package_share_directory('robotta_bringup'),
        'config',
        'robotta_controller.yaml'
        )

    return LaunchDescription([
      Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, test_controller],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    ])