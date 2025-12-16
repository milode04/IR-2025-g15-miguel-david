# IR-2025-g15-miguel-david
# Intelligent Robotics 2025 - Group 15 Repository

Welcome to the repository for the **Intelligent Robotics 2025** course (UPC-EEBE). This repository documents our progress through the lab sessions, covering everything from basic Linux tools to advanced ROS 2 simulation with sensors.

## 📄 Repository Description

This repository contains all the coursework, source code, and configuration files developed during the practical sessions. The project focuses on building a differential drive robot simulation from scratch using **ROS 2 (Humble)**, **URDF**, **RViz**, and **Gazebo**.

**Session 1:** Introduction to the development environment, Linux terminal mastery, Bash scripting, and Git/GitHub workflow.
**Session 2:** designing the robot's morphology using URDF and visualizing it in RViz.
**Session 3:** Organizing ROS 2 packages, creating launch files, and simulating the robot's physics and movement in Gazebo.
**Session 4:** Developing ROS 2 nodes (Publishers/Subscribers) with Python and integrating perception sensors (LiDAR and IMU) into the simulation.

## 📂 Repository Structure

```
ir_ws/
├── src/
│   ├── session1/                # Intro to Linux & Git
│   │   ├── welcome.sh           # Bash script for workspace greeting
│   │   └── robot_sensors.py     # Python script simulating ultrasonic sensors
│   │
│   ├── session2/                # URDF Modeling
│   │   └── first_robot.urdf     # Basic differential robot description
│   │
│   ├── session3/                    # ROS 2 Packages & Gazebo Simulation
│   │   ├── first_robot_description/ # Package containing URDF, meshes, and RViz config
│   │   │   ├── launch/              # Launch files for description
│   │   │   ├── rviz/                # RViz configuration files
│   │   │   └── urdf/                # Enhanced URDF with physics (inertias/collisions)
│   │   └── first_robot_bringup/     # Package for launching the complete simulation
│   │       └── launch/              # Main launch file for Gazebo + RViz
│   │
│   └── session4/                # ROS 2 Nodes & Sensors
│       ├── my_py_pkg/           # Basic Python nodes learning package
│       └── lidar_app/           # Exercise 1: Sensor simulation app
│           ├── lidar_publisher.py       # Node publishing simulated sensor data
│           └── decision_subscriber.py   # Node making decisions based on data
│
└── README.md                    # Project documentation
```
## How to Run the Code

### Session 1: Linux & Python Scripts

**1. Run the Bash Script:**
Navigate to the folder and execute the welcome script (ensure it has execution permissions `chmod +x`):
```
cd src/session1
./welcome.sh
```

**2. Run the Python Sensor Simulation:**
Execute the Python script that simulates the sensors:
```
python3 robot_sensors.py
```

### Session 2: Launching the URDF

To visualize the basic robot model created in Session 2 (visual geometry only) using the `urdf_tutorial` tools:

```
ros2 launch urdf_tutorial display.launch.py model:=src/session2/first_robot.urdf
```

### Session 3: RViz and Gazebo Simulation

**1. Visualizing in RViz:**
Launch the description package to see the model with valid physics properties in RViz:
```
ros2 launch first_robot_description display.launch.py
```

**2. Launching Simulation in Gazebo:**
Launch the physics simulator (Gazebo) along with RViz to spawn the robot:
```
ros2 launch first_robot_bringup fisrt_robot.launch.py
```
To control the robot, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard` in a separate terminal.

### Session 4: Nodes & Sensors

**1. Exercise 1: Publisher and Subscriber Nodes**
You need two terminals to run the application that simulates decision-making based on distance sensors.

*Terminal 1 (Publisher):*
```
ros2 run lidar_app lidar_publisher
```

*Terminal 2 (Subscriber):*
```
ros2 run lidar_app decision_subscriber
```
