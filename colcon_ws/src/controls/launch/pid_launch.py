from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false', description='Run in simulation mode')
    actions_arg = DeclareLaunchArgument('actions', default_value='false', description='Enable actions')
    
    # parameters - into a function since common except for name
    def create_pid_node(name, Kp, Ki, Kd, upper_limit, lower_limit, windup_limit):
        return Node(
            package="pid",
            executable="controller",
            name=name,
            parameters=[{
                "Kp": Kp,
                "Ki": Ki,
                "Kd": Kd,
                "upper_limit": upper_limit,
                "lower_limit": lower_limit,
                "windup_limit": windup_limit,
                "max_loop_frequency": 100.0,
                "min_loop_frequency": 100.0,
                "topic_from_controller": f"/controls/force/global/{name[0]}",
                "topic_from_plant": f"/state/{name[0]}",
                "setpoint_topic": f"/controls/pid/{name[0]}/setpoint",
                "pid_enable_topic": f"/controls/pid/{name[0]}/enable"
            }]
        )

    ## node groups
    # REAL
    real_pid_nodes = [
        create_pid_node("z_pid", 25, 10.0, 0.0, 1000, -1000, 100),
        create_pid_node("y_pid", 10, 0.3, 6.0, 10, -10, 10),
        create_pid_node("x_pid", 10, 0.3, 6.0, 10, -10, 10)
    ]

    quaternion_pid_real = Node(
        package="controls",
        executable="quaternion_pid.py",
        name="quaternion_pid",
        respawn=True,
        output="screen",
        parameters=[{"Kp": 7.0, "Ki": 0.37, "Kd": 0.8, "windup_limit": 30}]
    )
    
    # SIM
    sim_pid_nodes = [
        create_pid_node("z_pid", 20, 4.0, 5, None, None, None),
        create_pid_node("y_pid", 5, 0, 0.1, None, None, None),
        create_pid_node("x_pid", 5, 0, 0.1, None, None, None)
    ]
    
    quaternion_pid_sim = Node(
        package="controls",
        executable="quaternion_pid.py",
        name="quaternion_pid",
        respawn=True,
        output="screen",
        parameters=[{"Kp": 5.0, "Ki": 0, "Kd": 0.1, "windup_limit": 30}]
    )
    
    # ACTION
    action_pid_nodes = [
        create_pid_node("z_pid", 10, 0.7, 30.0, 25, -25, 25),
        create_pid_node("y_pid", 10, 0.7, 30.0, 12, -6, 6),
        create_pid_node("x_pid", 16.6, 0.15, 18, 14, -6, 6)
    ]
    
    quaternion_pid_action = Node(
        package="controls",
        executable="quaternion_pid.py",
        name="quaternion_pid",
        respawn=True,
        output="screen",
        parameters=[{"Kp": 0.2, "Ki": 0, "Kd": 0.2, "windup_limit": 30}]
    )
        
    
    return LaunchDescription([
        sim_arg,
        actions_arg,
        
        # REAL - if not sim and not actions
        GroupAction(real_pid_nodes + [quaternion_pid_real], condition=UnlessCondition(LaunchConfiguration("sim"))),
        
        # SIM - if sim
        GroupAction(sim_pid_nodes + [quaternion_pid_sim], condition=IfCondition(LaunchConfiguration("sim"))),
        
        # ACTION - if actions
        GroupAction(action_pid_nodes + [quaternion_pid_action], condition=IfCondition(LaunchConfiguration("actions")))
    ])
