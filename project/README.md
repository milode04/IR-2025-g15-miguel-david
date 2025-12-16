# Autonomous Rover Navigation - project_py_pkg

**Intelligent Robotics 2025 | Group 15**
**Authors:** Miguel & David

## Package Description

This ROS 2 package (`project_py_pkg`) contains the core autonomous navigation logic for our 6-wheeled differential drive rover. The main node, `project_first_node.py`, processes 2D LiDAR data to navigate corridors, centering the robot between walls and performing reactive obstacle avoidance.

## Project Structure

Based on the current package configuration:

```text
project_py_pkg/
├── project_py_pkg/
│   ├── __init__.py
│   └── project_first_node.py      # Main Autonomous Logic
├── resource/
├── test/
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```
## Algorithm Implementation & Design Choices

### 1. Implementation: How it Works
The autonomous navigation logic is implemented in the `ProjectFirstNode` class using **Python** and **ROS 2**. The system operates as a reactive controller that processes raw 2D LiDAR data to make real-time decisions. The workflow consists of three main stages:

#### **A. Sensor Pre-processing (The "Virtual Wall")**
Raw data from the `/scan` topic is filtered to ensure stability.
* **Sectoring:** The 360° LaserScan is divided into three critical sectors: **Front**, **Left**, and **Right**.
* **Clamping Mechanism:** To prevent the robot from reacting violently to distant open spaces (such as large intersections), we apply a saturation filter. Any distance reading greater than **1.5 meters** is capped (clamped) at 1.5m. This creates a "virtual corridor," forcing the robot to focus only on immediate obstacles and walls.

#### **B. Finite State Machine (Control Logic)**
The robot switches between two discrete states based on the `Front` distance reading:

* **State 1: Cruise & Center (PID Control)**
    * *Condition:* `Front Distance >= 0.65m`.
    * *Action:* The robot moves forward at a constant cruise speed (`0.25 m/s`). Simultaneously, a **PD Controller** continuously adjusts the angular velocity (`z`) to keep the robot centered.
    * *Logic:* It minimizes the error: `Error = Left_Distance - Right_Distance`.

* **State 2: Emergency Stop & Pivot**
    * *Condition:* `Front Distance < 0.65m`.
    * *Action:* The forward motion stops immediately. The robot evaluates the `Left` vs. `Right` average distances and performs a **Zero-Radius Turn** (spinning in place) towards the side with more available space.

---

### 2. Justification of Main Design Choices

#### **Why a PD Controller? (Kp=0.6, Kd=0.8)**
We selected a **Proportional-Derivative (PD)** controller instead of a simple Proportional (P) controller to achieve smoother navigation.
* **Kp = 0.6:** Provides enough reactivity to pull the robot away from walls quickly.
* **Kd = 0.8:** Acts as a "damper." Without the derivative term, the robot would oscillate (snake) down the corridor. The `Kd` term anticipates the error change and smooths out the steering commands.
* **Ki = 0.0:** The Integral term was intentionally set to zero. In corridor centering, steady-state error is rarely a major issue, and an integral term could introduce dangerous "windup" (overshooting) at corners.

#### **Why "Clamping" sensor values?**
In early tests, the robot would detect open doors or hallways 5+ meters away and aggressively steer towards them, causing instability. By limiting the "vision" to 1.5m, the robot ignores distant distractions and maintains a much more stable path in the center of the physical corridor.

#### **Why Open-Loop Turning?**
When an obstacle is detected directly in front, PID control is insufficient. We implemented a discrete "Stop & Turn" state because it is safer. By stopping `linear.x` completely, we utilize the robot's differential drive capability to spin without moving forward, eliminating the risk of clipping the wall during the turn.

---

### 3. Build & Run Instructions

To compile the project and run the autonomous navigation node, follow these steps in your ROS 2 workspace:

#### **Step 1: Build the Package**
Open Gazebo choosing the maze that you want (1, 2 or 3)
```bash
ros2 launch osr_bringup maze_simulation.launch.py maze:=maze_1.world
```

#### **Step 2: Build the Package**
Compile the specific package to ensure changes are applied, and run the node:
```bash
cd ~/ir_ws  # Navigate to your workspace root
colcon build 
source ~/.bashrc
ros2 run project_py_pkg project_first_node
```

## Conclusions

We are very satisfied with the final autonomous navigation solution we developed. The combination of a Finite State Machine for decision-making and a PD Controller for corridor centering has proven to be a robust strategy. The robot successfully navigates the environment, reacts correctly to walls, and avoids collisions autonomously.

However, there is still room for optimization. While the system is reliable, we would have liked to achieve a **higher cruising speed** while maintaining the same level of **stability**. During our testing, we observed that increasing the velocity often introduced oscillations that were difficult to dampen with the current controller tuning. Consequently, we prioritized safety and smoothness over raw speed for this final version. For instance, we think that only one group being able to run Gazebo at once made the project more dificult than initially thought.
