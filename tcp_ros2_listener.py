#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class TCPCommandListener(Node):
	def __init__(self):
		super().__init__('tcp_command_listener')
		self.publisher_ = self.create_publisher(String, 'voice_command', 10)
		self.start_tcp_server()


	def start_tcp_server(self):
		HOST='0.0.0.0'
		PORT=5005

		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
			s.bind((HOST, PORT))
			s.listen(1)
			self.get_logger().info(f'Listening on TCP {PORT}...')

			while True:
				conn, addr = s.accept()
				with conn:
					self.get_logger().info(f'Connection from {addr}')
					data = conn.recv(1024)
					if data:
						msg = String()
						msg.data = data.decode().strip()
						self.get_logger().info(f'Recieved: {msg.data}')
						self.publisher_.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = TCPCommandListener()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
