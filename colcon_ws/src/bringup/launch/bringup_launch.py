import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    # Declare the launch arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false', description='Run in simulation mode')
    bag_arg = DeclareLaunchArgument('bag', default_value='false', description='Enable bag recording')
    vision_arg = DeclareLaunchArgument('vision', default_value='false', description='Enable vision system')
    actions_arg = DeclareLaunchArgument('actions', default_value='false', description='Enable actions')
    ekf_arg = DeclareLaunchArgument('ekf', default_value='true', description='Enable EKF')

    # Include state_estimation launch file
    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("state_estimation"), '/launch/state_estimation_launch.py']),
        launch_arguments={'sim': LaunchConfiguration('sim'), 'ekf': LaunchConfiguration('ekf')}.items()
    )

    # Group for real sensors launch (when not in simulation)
    sensors_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("sensors"), '/launch/sensors_launch.py']),
                launch_arguments={'vision': LaunchConfiguration('vision')}.items()
            )
        ],
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    # Group for simulated sensors and depth republisher node (when in simulation)
    sensors_status_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("sensors"), '/launch/sensors_status_launch.py'])
            ),
            launch_ros.actions.Node(
                package='sensors',
                executable='depth_republisher',
                name='depth_republish',
                output='screen',
                parameters=[{'variance': 0.0}]
            )
        ],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # Include controls launch file
    controls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("controls"), '/launch/controls_launch.py']),
        launch_arguments={'sim': LaunchConfiguration('sim'), 'actions': LaunchConfiguration('actions')}.items()
    )

    # Include propulsion launch file
    propulsion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("propulsion"), '/launch/propulsion_launch.py']),
        launch_arguments={'sim': LaunchConfiguration('sim')}.items()
    )

    # Vision launch group (only if vision argument is true)
    vision_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch.substitutions.FindPackageShare("vision"), '/launch/vision_launch.py']),
                launch_arguments={'sim': LaunchConfiguration('sim')}.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('vision'))
    )

    # Assemble and return the full launch description
    return LaunchDescription([
        sim_arg,
        bag_arg,
        vision_arg,
        actions_arg,
        ekf_arg,
        state_estimation_launch,
        sensors_launch,
        sensors_status_launch,
        controls_launch,
        propulsion_launch,
        vision_launch
    ])
