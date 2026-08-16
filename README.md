# Design and Simulation of a Vision-Based Obstacle-Sensing System for Dynamic UAV Path Re-Planning

**Author:** Abdulkabeer Ismail Kolapo  
**Degree:** B.Eng Aeronautical and Astronautical Engineering, Kwara State University  

## 📌 Project Overview
This project presents a lightweight onboard sensor framework that pairs a custom-trained YOLOv8 vision model with a Rapidly-Exploring Random Tree (RRT) / Artificial Potential Field (APF) local planner. Designed and validated within a ROS 2, Gazebo, and MATLAB/Simulink architecture, the system bridges low-level flight control with high-level cognitive hazard response, allowing a UAV to instantly compute and execute collision-free bypass routes in dynamic, unmapped environments.

## 🛠️ Tech Stack & Tools
* **Flight Dynamics & Control:** MATLAB, Simulink
* **Simulation Environment:** Gazebo, ROS 2 (Ubuntu)
* **Computer Vision:** YOLOv8, Python, OpenCV
* **Algorithms:** Rapidly-Exploring Random Trees (RRT), Artificial Potential Fields (APF)

## 🚀 System Architecture & Methodology
The architecture is built on a unified **Sense → Decide → Re-Plan → Act** pipeline:
1. **Perception:** A custom YOLOv8 model detects hazards (Cars, Houses, Red Bricks, Trees) from a live Gazebo camera feed.
2. **Decision & Planning:** Detections trigger the APF/RRT planner inside Simulink to calculate lateral maneuvers or vertical updrafts based on obstacle height and proximity.
3. **Failsafe Watchdog:** SR-latched battery and mission-completion flags trigger a strict 25-meter climb and Return-to-Home (RTH) sequence.

![Simulink Architecture](Screenshot%202026-08-16%20142738.png)
*Figure 1: Simulink flight control logic and failsafe decision arbiter.*

## 📊 YOLOv8 Vision Model Performance
The custom YOLOv8 model was trained specifically to eliminate near/far ambiguity in dynamic environments. 

* **Classes:** Car, House, Red Brick, Tree
* **Confidence Threshold:** 76% optimal baseline.
* **Training Metrics:** The model achieved rapid convergence in Box Loss, Class Loss, and Object Loss, maintaining a high mAP score throughout the training epochs. 

| Confusion Matrix & Class Performance | Training Convergence |
| :---: | :---: |
| ![Confusion Matrix](Screenshot%202026-08-04%20165557.png)<br>![Performance by Class](Screenshot%202026-08-04%20165459.png) | ![Training Progress](Screenshot%202026-08-04%20164711.png) |

## 🚁 Simulation & Flight Validation
Validation was conducted using a 3D physics environment in Gazebo interfaced with ROS 2. 

### Autonomous Perception
The system successfully identifies obstacles in real-time, calculating bounding box areas to inflate configuration-space constraints. 
*Example: Red Brick detected and localized with a bounding box area of 35,838 px² triggering an immediate evasion protocol.*

![Gazebo Environment](Screenshot%202026-08-13%20142244.png) 
![Perception Bounding Box](Screenshot%202026-08-06%20124147.png)

### Telemetry & Flight Path Execution
The telemetry data confirms the successful execution of dynamic re-planning. The UAV maintains a standard cruise altitude of 15m. Upon detecting hazards on the flight path, the APF executes precise vertical updrafts (e.g., peaking at ~22.5m) to clear structures, and perfectly executes a 25m failsafe ceiling climb upon mission completion.

![3D Flight Telemetry](Screenshot%202026-08-14%20073548.png)
*Figure 2: MATLAB 3D Telemetry showing standard cruise, emergency vertical evasion, and the 25m RTH failsafe trigger.*
