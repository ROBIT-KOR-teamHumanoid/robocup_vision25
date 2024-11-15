from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='robocup_vision25',
      executable='robocup_vision25',
      output='screen'
    )
  ])
