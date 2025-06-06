from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        return LaunchDescription([
                Node(
                        package='llm_control_pkg',
                        executable='tcp_listener',
                        name='tcp_command_listener',
                        output='screen'
                ),
                Node(
                        package='llm_control_pkg',
                        executable='llm_command_executor',
                        name='command_executor',
                        output='screen'
                ),
                Node(
                        package='actuators_pkg',
                        executable='twist_vesc_LLM',
                        name='twist_vesc_llm',
                        output='screen'
                ),
               # Node(
               #         package='lidar_avoidance',
               #         executable='avoid_node',
               #         name='lidar_avoidance',
               #         output='screen'
               # ),
                Node(
                        package='lidar_avoidance',
                        executable='decision_node',
                        name='obstacle_decision',
                        output='screen'
                ),
        ])
