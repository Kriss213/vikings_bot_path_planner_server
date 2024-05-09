import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    TextSubstitution
)


def generate_launch_description():

    ### DATA INPUT ###
    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
        default_value="vikings_bot_1",
        description="Namespace of robot - [vikings_bot_1 or vikings_bot_2]"
    )

    use_sim_arg = DeclareLaunchArgument("use_sim",
        default_value="True",
        description='Use simulation or real time'
    )

    vikings_bot_name = LaunchConfiguration("vikings_bot_name")
    use_sim = LaunchConfiguration("use_sim")

    package_name = 'vikings_bot_path_planner_server'


    ### CONFIG FILES ###
    controller_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', PythonExpression(["'", vikings_bot_name, "_controller.yaml'"]) ])
    bt_navigator_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', PythonExpression(["'", vikings_bot_name, "_bt_navigator.yaml'"]) ])
    planner_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', PythonExpression(["'", vikings_bot_name, "_planner_server.yaml'"]) ])
    recovery_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', PythonExpression(["'", vikings_bot_name, "_recovery.yaml'"]) ])

    behavior = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'behavior.xml'])


    ### NODES ###
    controller = Node(
        namespace=vikings_bot_name,
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_yaml,
            {'use_sim_time': use_sim}
        ]
    )

    planner = Node(
        namespace=vikings_bot_name,
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_yaml,
            {'use_sim_time': use_sim}
        ]
    )
        
    behaviour = Node(
        namespace=vikings_bot_name,
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[
            recovery_yaml,
            {'use_sim_time': use_sim}
        ],
        output='screen',
    )

    navigator = Node(
        namespace=vikings_bot_name,
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_navigator_yaml,
            {"default_nav_to_pose_bt_xml": behavior},
            {'use_sim_time': use_sim}
        ]
    )

    lifecycle = Node(
        namespace=vikings_bot_name,
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[
            {'autostart': True},
            {'use_sim_time': use_sim},
            {'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]}
        ]
    )

    return LaunchDescription([
        vikings_bot_name_arg,
        use_sim_arg,

        SetParameter('use_sim_time',  use_sim),

        controller,
        planner,
        behaviour,
        navigator,
        lifecycle

    ])

