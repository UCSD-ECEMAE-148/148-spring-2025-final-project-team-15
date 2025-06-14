# ECE/MAE 148 - Team 15  
## Spring 2025 Final Project

<img style="width: 50%;" alt="Screenshot 2025-06-13 at 9 35 06 PM" src="https://github.com/user-attachments/assets/c1025ec2-f250-47fe-bab5-4b1ef2d3aeb1" />

---

### 🛠️ Project : Voice Controlled Autonomous Bot
<img style="width: 50%;" alt="Screenshot 2025-06-13 at 10 36 41 PM" src="https://github.com/user-attachments/assets/13300ca1-a6e6-4255-a381-541936cc506f" />


---

### 👥 Team Members:
- **Pranav Kambhampati** (ECE)  
- **Athena Wu** (ECE)  
- **Sebastian Castaneda** (ECE)  
- **Sujaan Mukherjee** (MAE)

---

## Project Overview / Proposal
The goal of our project is to build a voice-controlled robot, inspired by real-world applications of voice-controlled wheelchairs to asssit individuals with mobility impairments. Our robot recognizes and executes voice commands with LLM models and idetnfifies and avoid obstacles using lidars. The robot ecognizes and executes voice commands such as “forward”, “backward”, “stop.” The robot integrates speech recognition, sensor-based obstacle avoidance, and autonomous decision-making using ROS.

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
