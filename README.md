# ECE/MAE 148 - Team 15  
## Spring 2025 Final Project

<img style="width: 50%;" alt="Screenshot 2025-06-13 at 9 35 06 PM" src="https://github.com/user-attachments/assets/c1025ec2-f250-47fe-bab5-4b1ef2d3aeb1" />

---

### 🛠️ Project : Voice Controlled Autonomous Bot
<img style="width: 50%;" alt="Screenshot 2025-06-13 at 10 36 41 PM" src="https://github.com/user-attachments/assets/13300ca1-a6e6-4255-a381-541936cc506f" />


---

### 👥 Team Members:
- **Pranav Kambhampati** (ECE)  - Class of 2026
- **Athena Wu** (ECE)  - Class of 2026
- **Sebastian Castaneda** (ECE)  - Class of 2025
- **Sujaan Mukherjee** (MAE)  -  Class of 2025

---

## Project Overview / Proposal
The goal of our project is to build a voice-controlled robot, inspired by real-world applications of voice-controlled wheelchairs to assist individuals with mobility impairments. Our robot recognizes and executes voice commands with LLM models and identifies and avoids obstacles using lidars. The robot recognizes and executes voice commands such as “forward”, “backward”, “stop.” The robot integrates speech recognition, sensor-based obstacle avoidance, and autonomous decision-making using ROS.

### Goals We Have Accomplished:
- **Car/Vesc acts upon prompt input, control speed, duration and angle:** Successfully built a voice-controlled car that supports both lateral movements and turns. Decides rpm and motor spin time based on parameters recieved from the LLM.
- **LLM Voice Recognition using prompt to parse params:** We are successfully able to use an LLM to parse voice commands rather than a simple speech to text package. There is additional flexibility for what the user can say since the LLM pieces input into the right parameters.
- **Lidar Integration to Identify Obstacles:** We promised to have a car that wouldn't drive blidnly and would use the lidar to identify obstacles, which we accomplished. This lidar interfaces with the vesc node, providing it input on when and where to go.
- **Reaction Upon Obstacle Identification:** On top of the lidar detecting obstacles, we are able to successfully react to them. The car is able to steer clear of obstacles at low speeds.
- **Costmap Implementation in Obstacle Identification/Reaction:** We took our obstacle avoidance to the next level by implementing a costmap. The lidar is sectioned off into multiple sectors and calculates the 'risk' factor of driving into a specific sector. It ultimately tells the VESC to go in the direction where the 'risk' is minimized. This feature only kicks in when obstacles are detected.

### Nice to Haves/Bonus Features
- **Voice commands for more complex behaviors:** We initially planned to use OpenCV to tell the car commands like 'go around the blue obstacle,' but we weren't able to get to this level of processing.
- **Simplify speech patterns:** The LLM can only generate missing parameters from voice inputs to a certain extent. We were hoping to make this better but didn't get the chance to.
- **Smoother VESC operations:** The VESC is currently not super smooth, which goes against our ethos as a 'voice-controlled wheelchair.' We wanted to add more accleration/deceleration features so that the car speeds up and slows down smoother.

---

## Features of the Car
- Car/VESC acts upon prompt input, control speed, duration, angle
- LLM voice recognition - prompt to parse into params
- Lidar Integration to identify obstacle 
- Reaction upon obstacle identification (stop), provide feedback to user 
- More complex reaction upon obstacle identification (costmap)

---

## Demonstrations
Embed or link demo videos here:

- [Demo Video 1: Controlling the robot with voice command](https://youtube.com/shorts/sKy2-RqtOS4?feature=share)
- [Demo Video 2: Robot avoiding the wall on the left while not running into pedestrian on the right](https://youtu.be/NgpsvB51Ezo)
- [Demo Video 3: Robot going around an obstacle](https://youtu.be/sVomLy_VWWM)

---

## Node Architecture
Below, we show the architecture and structure of the nodes used to implemenet the robot. 
<img width="748" alt="Screenshot 2025-06-13 at 10 48 21 PM" src="https://github.com/user-attachments/assets/09c83e42-f5b6-4639-8134-80d30486547a" />

### Architecture Breakdown

Our current architecture includes the blue and yellow nodes and topics. We have not implemented the Command Manager Node.

Our system-critical nodes are:
- **TCP_Command_Listener:** This node runs a TCP Server using ROS2 to listen to TCP data sent from the laptop running the LLM. The LLM is responsible for sending over a direction, speed, duration and servo angle. The command listener is then responsible for introducing these parameters to the rest of the ROS2 network. If we think of the ROS2 system as a pipeline, the command listener node is all the way at the beginning.
- **LLM_Command_Executor:** This node is responsible for turning the parameters sent from the LLM into something that can be executed. This node publishes twist messages into the cmd_vel topic. These messages can then be used to control the VESC in terms of rpm and how long it spins for.
- **VESC_Twist:** This node is responsible for controlling the VESC. It directly interfaces with the VESC and sends rpm and angle commands from the twist messages it receives to control the VESC.

We also have additional logic in these nodes to publish stop twist messages to the VESC when nothing is being sent through cmd_vel to ensure that the VESC holds its position and a straight angle. We also use the duration parameter to ensure that the VESC runs for the right amount of time on commands.

In addition to these nodes, we simultaneously run additional nodes to help with obstacle avoidance. These include:
- **Lidar_Avoidance:** This node is responsible for processing input from the lidar data. It simply publishes stop messages when the car is close to running into something.
- **Smart_Decision:** This node is responsible for the costmap implementation for our obstacle avoidance. It handles the logic for calculating the 'risk factor' in each lidar sector and then publishing messages to the VESC through the lidar_status topic telling the robot which way to go. Namely, these messages include forward, left and right directions.

---

## Final Project Presentation
[Link to the slide](https://docs.google.com/presentation/d/16F6ugTsulLBXcFYbssiPY11haiGuQK-PavjkXhjKuz0/edit?usp=sharing)

---
## Acknowledgments
This project wouldn't have been possible if not for our teammates, Professor Silberman, and our two amazing TAs Alex and Winston. Thank you!


