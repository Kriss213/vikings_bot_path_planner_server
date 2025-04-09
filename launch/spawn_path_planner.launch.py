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

    vikings_bot_name = LaunchConfiguration("namespace").perform(context)
    use_sim = LaunchConfiguration("use_sim")

    package_name = 'vikings_bot_path_planner_server'

    ### CONFIG FILES ###
    controller_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_controller.yaml'])
    bt_navigator_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_bt_navigator.yaml' ])
    planner_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_planner_server.yaml'])
    recovery_yaml = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'vikings_bot_recovery.yaml'])
    behavior = PathJoinSubstitution([get_package_share_directory(package_name), 'config', 'behavior.xml'])

   
    ### NODES ###
    
    controller_node_n_params = GroupAction( # group to only set these params for this node
        actions=[
            SetParameter(
                name="global_frame",
                value=f"{vikings_bot_name}/odom" # add namespace to frame param
            ),
            SetParameter(
                name="robot_base_frame",
                value=f'{vikings_bot_name}/base_link' # add namespace to frame param,
            ),
            SetParameter(
                name="obstacle_layer.scan.topic", # for scan observation source
                value=f"/{vikings_bot_name}/lidar_scan"
                # for some reason this is not set from namespace (maybe because it is a nested node in controller manager?)
            ),
            Node(
                namespace=vikings_bot_name,
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
                name="robot_base_frame",
                value=f"{vikings_bot_name}/base_link" # add namespace to frame param
            ),
            SetParameter(
                name="obstacle_layer.scan.topic", # for scan observation source
                value=f"/{vikings_bot_name}/lidar_scan"
                # for some reason this is not set from namespace (maybe because it is a nested node in planner server?)
            ),
            Node(
                namespace=vikings_bot_name,
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                parameters=[
                    planner_yaml,
                    {'use_sim_time': use_sim},
                    #{'robot_base_frame': f'{vikings_bot_name}/base_link'} # add namespace to frame param
                ]
            )
        ]
    )

    behaviour = Node(
        namespace=vikings_bot_name,
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        respawn=True,
        output='screen',
        parameters=[
            recovery_yaml,
            {'use_sim_time': use_sim},
            {'robot_base_frame': f'{vikings_bot_name}/base_link'} # add namespace to frame param
        ],
    )

    navigator = Node(
        namespace=vikings_bot_name,
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        parameters=[
            bt_navigator_yaml,
            {"default_nav_to_pose_bt_xml": behavior},
            {'robot_base_frame': f'{vikings_bot_name}/base_link'}, # add namespace to frame param
            {'use_sim_time': use_sim}
        ]
    )

    lifecycle = Node(
        namespace=vikings_bot_name,
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

    return [controller_node_n_params,
        planner_node_n_params,
        behaviour,
        navigator,
        lifecycle]# + local_costmap_params + global_costmap_params

def generate_launch_description():

    ### DATA INPUT ###
    namespace_arg = DeclareLaunchArgument("namespace",
        default_value="vikings_bot_1",
        description="Namespace of robot"
    )

    use_sim_arg = DeclareLaunchArgument("use_sim",
        default_value="True",
        description='Use simulation or real time'
    )

    # vikings_bot_name = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim")

    # package_name = 'vikings_bot_path_planner_server'


    

    return LaunchDescription([
        namespace_arg,
        use_sim_arg,
        
        SetParameter('use_sim_time',  use_sim),

        OpaqueFunction(function=launch_setup)

    ])

