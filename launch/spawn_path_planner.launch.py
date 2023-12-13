import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def launch_setup(context, *arg, **args):

    ### DATA INPUT ###
    vikings_bot_name = LaunchConfiguration("vikings_bot_name").perform(context)

    package_name = 'vikings_bot_path_planner_server'


    ### CONFIG FILES ###
    controller_yaml = os.path.join(get_package_share_directory(package_name), 'config', f'{vikings_bot_name}_controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(package_name), 'config', f'{vikings_bot_name}_bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory(package_name), 'config', f'{vikings_bot_name}_planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory(package_name), 'config', f'{vikings_bot_name}_recovery.yaml')


    ### NODES ###
    controller = Node(
        namespace=vikings_bot_name,
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml])

    planner = Node(
        namespace=vikings_bot_name,
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml])
        
    behaviour = Node(
        namespace=vikings_bot_name,
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml],
        output='screen')

    navigator = Node(
        namespace=vikings_bot_name,
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml])

    lifecycle = Node(
        namespace=vikings_bot_name,
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'bt_navigator']}])
    
    return [
        controller,
        planner,
        behaviour,
        navigator,
        lifecycle
    ]


def generate_launch_description():

    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
            default_value="vikings_bot",
            description="Robot name to make it unique")
    

    return LaunchDescription([  
        vikings_bot_name_arg, 

        OpaqueFunction(function=launch_setup)

    ])
