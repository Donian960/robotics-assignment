# deterministic_simple_sim.py
# Copy-paste and run with: python deterministic_simple_sim.py
# Requires: networkx, pandas. Install with: pip install networkx pandas

import math
import json
from dataclasses import dataclass, field
from typing import List, Dict, Optional
import networkx as nx
import pandas as pd

# ---------- data models ----------
@dataclass
class Task:
    id: str
    pickup: str
    drop: str
    weight: float
    request_time: float = 0.0
    assigned_robot: Optional[str] = None
    assign_time: Optional[float] = None
    pickup_time: Optional[float] = None
    drop_time: Optional[float] = None
    status: str = "waiting"  # waiting, assigned, picked, delivered

@dataclass
class Robot:
    id: str
    node: str
    battery: float
    max_capacity: float
    speed: float
    state: str = "idle"  # idle, moving, charging (charging optional)
    path: List[str] = field(default_factory=list)
    path_progress: int = 0
    edge_progress: float = 0.0
    task_id: Optional[str] = None
    distance_travelled: float = 0.0
    energy_used: float = 0.0

# ---------- world ----------
class World:
    def __init__(self):
        self.G = nx.Graph()
        self.coords = {}
        self.chargers = set()
    def add_node(self, name, x, y, charger=False):
        self.G.add_node(name)
        self.coords[name] = (x,y)
        if charger:
            self.chargers.add(name)
    def add_edge(self, a, b):
        ax,ay = self.coords[a]; bx,by = self.coords[b]
        d = math.hypot(ax-bx, ay-by)
        self.G.add_edge(a, b, length=d)
    def shortest_path(self, a, b):
        return nx.shortest_path(self.G, a, b, weight='length')
    def path_length(self, path):
        if not path or len(path)<2: return 0.0
        return sum(self.G[path[i]][path[i+1]]['length'] for i in range(len(path)-1))
    def nearest_charger(self, node):
        best = None; bd = float('inf')
        for c in self.chargers:
            try:
                p = self.shortest_path(node, c)
                d = self.path_length(p)
                if d < bd:
                    bd = d; best = c
            except nx.NetworkXNoPath:
                pass
        return best, bd

# ---------- server / allocator ----------
class Server:
    def __init__(self, world: World, robots: Dict[str, Robot], tasks: Dict[str, Task]):
        self.w = world
        self.robots = robots
        self.tasks = tasks

    def compute_cost(self, robot: Robot, task: Task):
        # cost: purely distance (deterministic)
        try:
            p1 = self.w.shortest_path(robot.node, task.pickup)
            p2 = self.w.shortest_path(task.pickup, task.drop)
        except nx.NetworkXNoPath:
            return float('inf')
        d = self.w.path_length(p1) + self.w.path_length(p2)
        # simple capacity check: infeasible if weight > robot capacity
        if task.weight > robot.max_capacity:
            return float('inf')
        return d

    def allocate_all_waiting(self, current_time: float, logs: list):
        # deterministic: allocate waiting tasks in order, only to idle robots
        for t in self.tasks.values():
            if t.status != 'waiting':
                continue
            best = None
            best_cost = float('inf')
            for r in self.robots.values():
                if r.state == 'moving':  # only idle robots considered
                    continue
                c = self.compute_cost(r, t)
                if c < best_cost:
                    best_cost = c; best = r
            if best is None or best_cost == float('inf'):
                continue
            # assign
            t.assigned_robot = best.id
            t.assign_time = current_time
            t.status = 'assigned'
            p1 = self.w.shortest_path(best.node, t.pickup)
            p2 = self.w.shortest_path(t.pickup, t.drop)
            best.path = p1 + p2[1:]
            best.path_progress = 0; best.edge_progress = 0.0; best.task_id = t.id; best.state = 'moving'
            logs.append({'time': current_time, 'event': 'assigned', 'task': t.id, 'robot': best.id, 'path': best.path})

# ---------- helpers & run ----------
def create_world():
    w = World()
    # nodes placed 10 meters apart: A-B-C-D
    w.add_node('A', 0, 0, charger=True)  # charger at A (optional usage)
    w.add_node('B', 10, 0)
    w.add_node('C', 20, 0)
    w.add_node('D', 30, 0)
    w.add_edge('A', 'B'); 
    w.add_edge('B', 'C'); 
    w.add_edge('C', 'D')
    return w

def create_robots():
    # deterministic starting nodes and parameters
    return {
        'r1': Robot(id='r1', node='A', battery=80.0, max_capacity=2.0, speed=1.0),
        'r2': Robot(id='r2', node='C', battery=60.0, max_capacity=1.0, speed=0.8)
    }

def create_tasks():
    # hardcoded tasks (no randomness)
    return {
        't1': Task(id='t1', pickup='B', drop='D', weight=0.5),
        't2': Task(id='t2', pickup='A', drop='C', weight=0.5),
        't3': Task(id='t3', pickup='D', drop='A', weight=0.5)
    }

def move_robot(robot: Robot, world: World, dt=1.0):
    if robot.state != 'moving' or not robot.path:
        return
    if robot.path_progress >= len(robot.path) - 1:
        return
    cur = robot.path[robot.path_progress]; nxt = robot.path[robot.path_progress + 1]
    edge_len = world.G[cur][nxt]['length']
    travel = robot.speed * dt
    remaining = edge_len - robot.edge_progress
    if travel < remaining:
        robot.edge_progress += travel
        robot.distance_travelled += travel
        robot.energy_used += travel * 0.2
        robot.battery = max(0.0, robot.battery - travel * 0.2)
    else:
        robot.distance_travelled += remaining
        robot.energy_used += remaining * 0.2
        robot.battery = max(0.0, robot.battery - remaining * 0.2)
        robot.node = nxt
        robot.path_progress += 1
        robot.edge_progress = 0.0

def run_sim(dt=1.0):
    world = create_world()
    robots = create_robots()
    tasks = create_tasks()
    server = Server(world, robots, tasks)
    logs = []
    time = 0.0
    # initial allocation
    server.allocate_all_waiting(time, logs)

    # run until all tasks delivered
    while True:
        # move robots
        for r in robots.values():
            move_robot(r, world, dt)
            # arrival at pickup
            if r.task_id:
                t = tasks[r.task_id]
                if t.status == 'assigned' and r.node == t.pickup:
                    t.pickup_time = time
                    t.status = 'picked'
                    logs.append({'time': time, 'event': 'pickup', 'task': t.id, 'robot': r.id})
                # arrival at drop
                if t.status == 'picked' and r.node == t.drop:
                    t.drop_time = time
                    t.status = 'delivered'
                    logs.append({'time': time, 'event': 'drop', 'task': t.id, 'robot': r.id})
                    r.task_id = None
                    r.state = 'idle'
        # try to allocate any remaining waiting tasks to idle robots
        server.allocate_all_waiting(time, logs)
        # termination check
        all_delivered = all(t.status == 'delivered' for t in tasks.values())
        if all_delivered:
            break
        time += dt
        if time > 10000.0:
            raise RuntimeError("Simulation timeout - something went wrong")
    # present results
    logs_df = pd.DataFrame(logs)
    tasks_df = pd.DataFrame([{
        'task': t.id, 'pickup': t.pickup, 'drop': t.drop, 'assign_time': t.assign_time,
        'pickup_time': t.pickup_time, 'drop_time': t.drop_time, 'status': t.status, 'assigned_robot': t.assigned_robot
    } for t in tasks.values()])
    robots_df = pd.DataFrame([{
        'robot': r.id, 'end_node': r.node, 'battery_end': r.battery,
        'distance': r.distance_travelled, 'energy_used': r.energy_used
    } for r in robots.values()])
    metrics = {'sim_time': time, 'num_tasks': len(tasks_df), 'delivered': sum(1 for t in tasks.values() if t.status == 'delivered')}
    print("Metrics:"); print(json.dumps(metrics, indent=2))
    print("\nTasks:"); print(tasks_df.to_string(index=False))
    print("\nRobots:"); print(robots_df.to_string(index=False))
    if not logs_df.empty:
        print("\nLogs:"); print(logs_df.to_string(index=False))
    return {'metrics': metrics, 'tasks': tasks_df, 'robots': robots_df, 'logs': logs_df}

if __name__ == "__main__":
    run_sim()
