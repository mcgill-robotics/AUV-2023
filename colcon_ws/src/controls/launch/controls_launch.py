from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false', description='Run in simulation mode')
    actions_arg = DeclareLaunchArgument('actions', default_value='false', description='Enable actions')
    
    # parameters 
    params = {
        "min_safe_goal_depth": -4,
        "max_safe_goal_depth": -0.5,
        "time_to_settle": 1,
        "pid_positional_tolerance": 0.2,
        "pid_quaternion_w_tolerance": 0.97,
        "settle_check_rate": 10,
        "superimposer_loop_rate": 10,
    }


    # include pid_launch.py
    pid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("controls"), '/launch/pid_launch.py']),
        launch_arguments={"sim": LaunchConfiguration('sim'), "actions": LaunchConfiguration('actions')}.items(),
    )

    # nodes
    superimposer_node = Node(
        package="controls",
        executable="superimposer.py",
        name="superimposer",
        respawn=True,
        output="screen"
    )

    servers_node = Node(
        package="controls",
        executable="init_servers.py",
        name="servers",
        respawn=False,
        output="screen"
    )

    return LaunchDescription([
        sim_arg,
        actions_arg,
        pid_launch,
        superimposer_node,
        servers_node,
    ])

