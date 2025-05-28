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
		)

	])
