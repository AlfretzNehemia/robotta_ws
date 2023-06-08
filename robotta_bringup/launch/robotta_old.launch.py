import os
# import xacro

# from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Get the launch configuration


    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="start RViz automatically with the launch file",
    )

    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('robotta_bringup'), 'launch', 'rsp.launch.py' 
    #     )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control':'true'}.items()
    # )

    # Get URDF via xacro
    # use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robotta_description"), "urdf", "robotta.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('robotta_description'),
    #     'rviz',
    #     'robotta.rviz')

    # Get the robot param configuration
    # robot_controller_config = os.path.join(get_package_share_directory('robotta_bringup'), 'config', 'robotta_controller.yaml')
    # teleop_config = os.path.join(get_package_share_directory('robotta_teleop'), 'config', 'teleop_config.yaml')

    robot_controller_config = PathJoinSubstitution(
        [
            FindPackageShare("robotta_bringup"),
            "config",
            "robotta_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("robotta_description"),
            "rviz",
            "robotta.rviz"
        ]
    )

    # teleop_config = PathJoinSubstitution(
    #     [
    #         FindPackageShare("robotta_teleop"),
    #         "config",
    #         "teleop_config.yaml",
    #     ]
    # )


    # controller_manager_node = Node(
    #     package="robotta_manager",
    #     executable="robotta_manager_node",
    #     parameters=[{"robot_description": robot_description_content},
    #                  robot_controller_config]
    #     # output={
    #     #     "stdout": "screen",
    #     #     "stderr": "screen",
    #     # },
    # )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controller_config],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )    

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager_node])


    diff_drive_spawner = Node(
        package= "controller_manager",
        executable="spawner.py",
        arguments=["robotta_drive_controller"],
    )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager_node,
    #         on_start = [diff_drive_spawner], 
    #     )
    # )

    joint_state_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_controller"],
    )

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager_node,
    #         on_start = [joint_state_broad_spawner], 
    #     )
    # )
    

    # robotta_teleop_node = Node(
    #     package="robotta_teleop",
    #     executable="robotta_teleop",
    #     parameters=[teleop_config]
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            arg_show_rviz,
            # node_robot_state_publisher,
            # controller_manager_node,
            # robotta_teleop_node,
            # rviz_node,
            # rsp,
            # delayed_controller_manager,
            # delayed_diff_drive_spawner,
            # delayed_joint_broad_spawner
            robot_state_pub_node,
            controller_manager_node,
            diff_drive_spawner,
            joint_state_broad_spawner,
            rviz_node
        ]
    )