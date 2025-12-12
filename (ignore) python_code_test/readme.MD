
## Objective
This simulation is written in pure Python and requires a few common scientific/data packages. This is the base code for the server logic

Run the Simulation:
Execute the main Python script. The simulation will run until all tasks are delivered and will print final metrics and logs to the console.

## Core Simulation Details
The World: The environment is a simple, straight-line graph

Nodes: A, B, C, D (A has a charging station).
Edges: All edges have a length of 10 meters.

Allocation Logic (The Server):
The server uses a simple distance check to assign priority
Feasibility Check (Capacity): Is the task weight within the robot's max_capacity? If not assign infite penalty

Cost Check (Distance): If viable, the robot with the shortest total travel path (Pickup + Drop) is chosen.

We run the robot in increments where in dt time if it can move on the path we do it if no obstacele else move till the obstacle
