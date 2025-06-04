# 🚗 Vehicle to Infrastructure Communication for Autonomous Navigation

This repository contains the complete implementation and documentation for my Bachelor's thesis project titled:

> **"Vehicle to Infrastructure (V2I) Communication for Autonomous Navigation"**

Developed using **Robot Operating System (ROS Noetic)** and **Gazebo**, this project demonstrates how autonomous robots can achieve safe, intelligent, and infrastructure-aware navigation in simulated environments using A* pathfinding, SLAM, and real-time sensor feedback.

---

## 📖 Project Overview

This project centers around a **four-wheel differential drive robot** navigating autonomously in a structured environment with **real-time response to dynamic obstacles and traffic signals (V2I)**. A* algorithm drives the path planning, SLAM builds a live map, and traffic light states guide decision-making. The robot is fully simulated in a **Gazebo 11** environment and controlled using ROS topics, services, and nodes.

---

## 🚀 Key Features

- 🧭 **Autonomous Path Planning** using A* algorithm with obstacle cost functions
- 📡 **Vehicle-to-Infrastructure (V2I) Communication** with simulated traffic lights
- 🛑 **Dynamic Obstacle Detection & Replanning** using laser scan data
- 🗺️ **Mapping and Localization** via GMapping/Hector SLAM
- ⚙️ **Realistic Robot Simulation** in custom Gazebo world with LiDAR, camera, and ROS plugins
- 📊 **Real-time Feedback & Logging** of metrics like path length, movement accuracy, and compliance

---

## 🧠 Algorithm Logic (Visualized as Pseudocode)

### 🧩 Algorithm 1 – A* Pathfinding & Navigation Control

procedure AStarPathFinder
    Initialize ROS node and topic subscribers (/map, /odom, /goal, /scan, /traffic_light_state)
    Set robot state and movement cost parameters

    while node is running:
        If map data: update internal grid map
        If odometry: update robot pose
        If goal: initialize pathfinding
        If laser scan: detect obstacles
        If traffic light: update state and modify behavior

        Perform A* Search:
            Use priority queue for optimal path
            Maintain came_from and cost_so_far dictionaries
            Apply heuristic + real-world cost (turns, buffer, dynamic obstacles)

        Execute movement:
            Send motion commands to /cmd_vel
            Recalculate path if dynamic obstacle encountered
            Log metrics (path length, goal accuracy, stop time)
end

procedure RotateAndMove(path)
    Record start time
    Initialize path length = 0

    for each waypoint:
        If goal reached: stop and log
        If traffic light is RED: wait(3s)
        If obstacle detected: call HandleDynamicObstacle()

        rotateTo(waypoint direction)
        moveStraightTo(waypoint)
        Log position, update metrics
    end for

    Stop movement
    Record end time
    Calculate accuracy
    Save results to log file
end
text

# Step 1: Create your catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Step 2: Clone this repository
git clone https://github.com/jeevanprakashhaprojects/AutonomousVehicle.git

# Step 3: Build the workspace
cd ~/catkin_ws
catkin_make

# Step 4: Source the workspace
source devel/setup.bash


# Launch the Gazebo simulation with robot and world
roslaunch navbot_description simulation.launch

# Launch the navigation stack (SLAM + A*)
roslaunch navbot_navigation navigation.launch

# Launch V2I communication (traffic light publisher)
roslaunch v2i_communication traffic_light.launch


# Architecture Diagram
![image](https://github.com/user-attachments/assets/7fcae246-d511-41cb-bd8c-6a07d5ff7a5c)

# A* Algorithm when giving Goal State it is finding shortest Path

![image](https://github.com/user-attachments/assets/c546bf54-40cf-4011-b8c7-87d1b1ac8305)

# Plot Showing that the Path is hitting an laser scan data

![image](https://github.com/user-attachments/assets/f60c1fe2-313f-40cf-ba2b-f287e6002bc4)

# When Obstacle Detected by Robot it stops recalculate the path by taking that laser scan data as occupied

![image](https://github.com/user-attachments/assets/b4c7ecfd-e3e0-4b5c-bc27-32e779c5c451)

# Gazebo Initialization

![image](https://github.com/user-attachments/assets/c3d3e563-e6c2-4f2a-8ca3-4da9e870b31b)

# METRICS CALCULATION

![image](https://github.com/user-attachments/assets/82cb79f5-1c81-49f2-9101-c0b700d2be31)

## 🔚 Conclusion

This project developed an autonomous navigation system for a four-wheel differential drive robot using ROS and Gazebo, integrating A* path planning and SLAM (Gmapping). The system performed effectively in static environments, successfully calculating optimal paths and navigating with precision. However, real-time performance in dynamic scenarios revealed challenges in obstacle detection and map updating, highlighting the need for improved sensor processing and responsiveness.

Despite these limitations, the project demonstrated a functional integration of planning, mapping, and V2I communication, contributing meaningful insights into ROS-based autonomous systems. It stands out for its practical implementation, focused on differential drive modeling and sensor fusion, offering a strong foundation for future development and enhancement.










