from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PathJoinSubstitution
)

def launch_setup(context, *args, **kwargs):

    use_sim = False # CRUCIAL! Otherwise "sim time" will not tick unless Gazebo is runnning.

    package_name = 'vikings_bot_path_planner_server'

    ### CONFIG FILES ###
    controller_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_controller.yaml'])
    bt_navigator_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_bt_navigator.yaml' ])
    planner_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_planner_server.yaml'])
    recovery_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_recovery.yaml'])
    behavior = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'behavior.xml'])

   
    ### NODES ###
    controller_node_n_params = GroupAction( # not actually needed but needs to be launched
        actions=[
            SetParameter(
                name='map_topic',
                value=f"/map"
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=True,
                parameters=[
                    controller_yaml,
                    {'use_sim_time': use_sim}
                ]
            )
        ]
    )   
    planner_node_n_params = GroupAction( # group to only set these params for this node
        actions=[

            SetParameter(
                name='map_topic',
                value=f"/map"
            ),
            SetParameter(
                name='courier_obstacle_layer.enabled',
                value='True'
            ),
            SetParameter(
                name='update_frequency',
                value=10.0
            ),
           
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                parameters=[
                    planner_yaml,
                    {'use_sim_time': use_sim}
                ]
            )
        ]
    )

    behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        respawn=True,
        output='screen',
        parameters=[
            recovery_yaml,
            {'use_sim_time': use_sim},
        ],
    )

    navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        parameters=[
            bt_navigator_yaml,
            {"default_nav_to_pose_bt_xml": behavior},
            {'use_sim_time': use_sim}
        ]
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        respawn=True,
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

    return [
        controller_node_n_params,
        planner_node_n_params,
        behaviour,
        navigator,
        lifecycle]

def generate_launch_description():


    use_sim_arg = DeclareLaunchArgument("use_sim",
        default_value="True",
        description='Use simulation or real time'
    )
    use_sim = LaunchConfiguration("use_sim")   

    return LaunchDescription([
        #namespace_arg,
        use_sim_arg,
        
        SetParameter('use_sim_time',  use_sim),

        OpaqueFunction(function=launch_setup)

    ])

