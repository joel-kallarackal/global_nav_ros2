from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "global_navigator", package='global_nav', executable='waypoint_navigator', output='screen'),
    ])