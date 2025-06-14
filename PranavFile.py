import socket

def send_command_to_jetson(command, jetson_ip, port=5005):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((jetson_ip, port))
        s.sendall(command.encode())
        print(f"Sent command: {command}")

if __name__ == "__main__":
    jetson_ip = "192.168.10.245"  # Replace with your Jetson's IP address
    command = "move forward"
    send_command_to_jetson(command, jetson_ip)


"""
If we want to send it using bash echo command: echo "sup" | nc 192.168.113.127 5005
nc indicates netcat, follwed by Jetson IP and port
"""