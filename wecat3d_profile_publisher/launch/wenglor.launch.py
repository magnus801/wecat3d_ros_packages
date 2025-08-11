from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wecat3d_profile_publisher',
            executable='profile_make',
            name='profile_make',
            output='screen'
        ),
        Node(                               # ← extension kept
            package='wecat3d_profile_publisher',
            executable='drishti_odometry.py',
            name='drishti_odometry',
            output='screen'
        ),
        Node(                               # ← extension kept
            package='wecat3d_profile_publisher',
            executable='combined_wenglor.py',
            name='combined_wenglor',
            output='screen'
        ),
    ])
