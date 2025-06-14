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

---

## Final Project Presentation
[Link to the slide](https://docs.google.com/presentation/d/16F6ugTsulLBXcFYbssiPY11haiGuQK-PavjkXhjKuz0/edit?usp=sharing)
