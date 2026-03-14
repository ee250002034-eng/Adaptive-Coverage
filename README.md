
# Decentralized Adaptive Voronoi Coverage

## Overview
This project implements a decentralized multi-robot coverage algorithm using Voronoi partitioning. The goal of the system is to allow multiple robots to autonomously divide an environment and perform efficient area coverage without centralized coordination.

Each robot computes a Voronoi cell based on the positions of neighboring robots and moves toward the centroid of its assigned region. As robot positions change, the Voronoi regions are continuously updated, enabling adaptive and efficient coverage of the environment.

The implementation also includes additional mechanisms such as adaptive exploration, velocity saturation, and decentralized decision making, which improve the robustness and scalability of the system.

This project demonstrates how decentralized algorithms can be used for coordinated robotic exploration and can be extended to applications such as environmental monitoring, surveillance, and search-and-rescue operations.

---

## Key Features

- Decentralized Control  
  Robots make decisions based only on local information without relying on a central controller.

- Voronoi-based Partitioning  
  The environment is dynamically divided among robots using Voronoi regions.

- Centroid-based Motion Control  
  Each robot moves toward the centroid of its Voronoi cell to maximize coverage efficiency.

- Adaptive Exploration  
  Robots explore new regions when coverage stagnates or when unexplored areas are detected.

- Velocity Saturation  
  Robot velocities are bounded to maintain stable and realistic motion behavior.

- Scalable Multi-Robot System  
  The algorithm naturally scales with an increasing number of robots.

---

## System Workflow

The system operates in a continuous loop:

1. Robots broadcast their current positions.
2. Voronoi partitioning is computed using the robot positions.
3. Each robot determines the centroid of its Voronoi region.
4. The robot generates a control input directing it toward the centroid.
5. Velocity saturation limits the robot's speed to maintain stability.
6. Exploration behavior activates if the robot detects insufficient coverage progress.

---

## Repository Structure

adaptive_coverage_ws

src/  
└── adaptive_coverage/  
  ├── coverage_node.py  
  ├── controller.py  
  ├── exploration.py  
  └── utils.py  

launch/  
└── coverage_simulation.launch.py  

worlds/  
└── simulation_world.world  

README.md

---

## Installation

### Prerequisites

- ROS2 Humble
- Python 3
- Colcon build system

### Clone the Repository

git clone https://github.com/yourusername/adaptive-voronoi-coverage.git  
cd adaptive-voronoi-coverage

### Build the Workspace

colcon build

### Source the Workspace

source install/setup.bash

---

## Running the Simulation

ros2 launch adaptive_coverage coverage_simulation.launch.py

Once launched, the robots will begin dividing the environment using Voronoi partitioning and move toward their respective regions to perform decentralized coverage.

---

## Results

The simulation demonstrates:

- Decentralized coordination between robots
- Dynamic Voronoi region updates
- Stable centroid-based robot motion
- Adaptive exploration of uncovered regions

The algorithm successfully distributes robots across the environment, enabling efficient coverage without centralized control.

---

## Future Improvements

Possible future extensions include:

- Integration of obstacle avoidance
- Implementation on physical robots
- Improved exploration strategies
- Communication constraints and realistic sensing models

---

## Author

Kritharth T

