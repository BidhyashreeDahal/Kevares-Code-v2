from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perimeter_dir_arg = DeclareLaunchArgument(
        'perimeter_dir',
        default_value='/home/user/bunker_ws/src/bunker_core/bunker_core/perimeter',
        description='Shared folder for lawn perimeter/path/waypoint data. Defaults to the robot path. Override with your laptop path for local testing.'
    )
    path_dir_arg = DeclareLaunchArgument(
        'path_dir',
        default_value='/home/user/bunker_ws/src/bunker_core/bunker_core/path',
        description='Folder cargohauling_navigation_node reads waypoints from. Defaults to the robot path. Currently separate from perimeter_dir — see flagged note for Abdeali.'
    )
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface name.'
    )

    perimeter_dir = LaunchConfiguration('perimeter_dir')
    path_dir = LaunchConfiguration('path_dir')
    can_interface = LaunchConfiguration('can_interface')

    return LaunchDescription([
        perimeter_dir_arg,
        path_dir_arg,
        can_interface_arg,

        # === bunker_hardware ===
        Node(package='bunker_hardware', executable='gps_node', name='gps_node', output='screen'),
        Node(package='bunker_hardware', executable='can_feedback_node', name='can_feedback_node',
             output='screen', parameters=[{'can_interface': can_interface}]),
        Node(package='bunker_hardware', executable='robot_controller_node', name='robot_controller_node',
             output='screen', parameters=[{'can_interface': can_interface}]),
        Node(package='bunker_hardware', executable='realsense_publisher_node', name='realsense_publisher_node', output='screen'),
        Node(package='bunker_hardware', executable='realsense_left_publisher_node', name='realsense_left_publisher_node', output='screen'),
        Node(package='bunker_hardware', executable='realsense_right_publisher_node', name='realsense_right_publisher_node', output='screen'),

        # === bunker_navigation ===
        Node(package='bunker_navigation', executable='path_perimeter_node', name='path_perimeter_node',
             output='screen', parameters=[{'perimeter_dir': perimeter_dir}]),
        Node(package='bunker_navigation', executable='path_generator_node', name='path_generator_node',
             output='screen', parameters=[{'perimeter_dir': perimeter_dir}]),
        Node(package='bunker_navigation', executable='stanlynavigation_node', name='stanlynavigation_node',
             output='screen', parameters=[{'perimeter_dir': perimeter_dir}]),
        Node(package='bunker_navigation', executable='cargo_path_perimeter_node', name='cargo_path_perimeter_node',
             output='screen', parameters=[{'perimeter_dir': perimeter_dir}]),
        Node(package='bunker_navigation', executable='cargohauling_navigation_node', name='cargohauling_navigation_node',
             output='screen', parameters=[{'path_dir': path_dir}]),

        # === bunker_tools ===
        Node(package='bunker_tools', executable='discord_zigzag_test_node', name='discord_zigzag_test_node',
             output='screen', parameters=[{'perimeter_dir': perimeter_dir}]),
        Node(package='bunker_tools', executable='gps_logger_node', name='gps_logger_node',
             output='screen', parameters=[{'perimeter_dir': perimeter_dir}]),

        # Intentionally NOT included:
        # object_detection_node     — confirmed by Abdeali not currently in use.
        #                             exp.pt also has an unresolved ultralytics
        #                             version/fork mismatch (neither 8.2.96 nor
        #                             8.3.153 can load it) if revisited later.
        # imu_node                  — commented out in the original
        # dodging_node               — commented out in the original
        # collision_avoidance_node  — was never active in the original; also has an
        #                             unresolved movement_commands conflict, needs
        #                             motion authority arbitration before activation
    ])