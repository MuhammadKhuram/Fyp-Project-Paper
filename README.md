# Autonomous Weed Removal Robot - Simulation & Modeling

This repository contains the software simulation, kinematic modeling, and vision AI for an autonomous weeding robot. 

## How the Code is Structured
We are using 4 core Python files.

* **`delta_robot_kinematics.py`**: Generates 3D workspace and joint angle plots.
* **`mobile_robot_navigation.py`**: Generates path planning and EKF error plots.
* **`yolov8_weed_detector.py`**: Evaluates YOLOv8 performance and saves bounding-box images.
* **`main_robot_controller.py`**: Simulates the full robotic mission and logs battery/latency metrics.

## Setup
1. Clone the repository.
2. Install dependencies: `pip install -r requirements.txt`
3. Run any of the 4 python files to generate outputs.