# Energy-Aware Combinatorial Auctions for Multi-Robot Logistics  
**Course:** Intelligent Robotics (University of Birmingham)  
**Simulator:** Webots
**Language:** Python 3 

## Project Overview

This project implements a centralized supervisory framework for real-time task allocation in a warehouse environment. Both **Sequential Single-Item(SSI)** and a advanced **Combinatorial Auctions** are implemented.

The system is **battery- and energy-aware**, using an exponential risk term that encourages robots to proactively seek charging before depletion. It also incorporates **Online Parameter Estimation** to learn the true robot velocity and battery discharge rates during simulation, improving allocation accuracy in non-ideal physics.

Please note the repository got quite messy during development. The world file to be assessed is at the path:

dani's folder no entry/worlds/.linefollower.wbproj


## Statement of Contribution

Per assignment guidelines, the following outlines what was **self-implemented** versus **pre-programmed**.

### 1. Self-Implemented 

- **Combinatorial Auction Engine**  
  Bundle generation (up to size `g = 3`), localized TSP solver (Dynamic Programming), and greedy winner selection.

- **Objective Cost Function**  
  Integrated time cost, battery recovery expectations, and the exponential battery-risk penalty:  

- **System Architecture**  
  Full Client–Supervisor–Robot design and a custom JSON-based communication protocol.

- **Online Learning Logic**  
  Exponential Moving Averages (EMA) for estimating robot speed and discharge rate.

- **Priority Buffering**  
  “Maturity Check” mechanism that delays low-priority tasks.

- **Robot Finite State Machine (FSM)**  
  States: `IDLE`, `BUSY`, `MOVING_TO_CHARGE`, `CHARGING`.

- **Line Following Algorithm**  
  Uses the robot's camera in combination with a state system consisting of `IDLE`, `FOLLOW`, `TURN`, and `STOPPING` to control the speed of the motors.

- **Collision Avoidance Algorithm & Odometry**  
  Uses the robot's distance sensors to detect collisions and two states `AVOIDING` and `BLOCKED INTERSECTION` to avoid them.

### 2. Pre-Programmed Packages & Libraries

- **Webots Libraries**  
  Used the standard Cyberbotics robot and motor API (e.g. `setVelocity()`).

- **K-Team's Khepera IV**  
  Two-wheeled robot configured with various nodes.
  * PROTO modified by us to set the `redColorSensitivity` of the Infrared Sensors to `0`.

- **K-Team's Khepera III-Gripper**  
  Robot arm used for manipulation of objects.

- **NetworkX**  
  Used as a graph structure and for `dijkstra_path_length`.  
  *Only for shortest path calculation*

- **Python Standard Library**  
  `time`, `json`, `math`, `statistics`, `re`, `heapq`, `itertools`, `collections`, `dataclasses`, `typing`.



## Installation & Prerequisites

### 1. Install NetworkX

This project requires `networkx` for graph traversal. Find the python path by going to webots prefrences and look for python command, then at that path run the following command.

```bash
pip install networkx
```
