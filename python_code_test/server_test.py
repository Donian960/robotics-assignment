# Simulation of Multi-Robot Market-Based Task Allocation with Charging Stations
# Runs a time-stepped simulation entirely in Python.
# Outputs logs and summary metrics as DataFrame for inspection.
# Requires networkx, numpy, pandas, matplotlib (matplotlib will not specify colors).

import math, random, time, json
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import networkx as nx
import numpy as np
import pandas as pd

random.seed(1)
np.random.seed(1)

# -----------------------------
# Models
# -----------------------------

@dataclass
class Task:
    id: str
    pickup: str
    drop: str
    weight: float
    priority: int
    request_time: float
    assigned_robot: Optional[str] = None
    assign_time: Optional[float] = None
    pickup_time: Optional[float] = None
    drop_time: Optional[float] = None
    status: str = "waiting"  # waiting, assigned, picked, delivered, cancelled

@dataclass
class Robot:
    id: str
    node: str
    pos_xy: Tuple[float,float]  # exact coordinates for distance calcs
    battery: float  # percentage 0-100
    load_weight: float  # current carried weight
    max_capacity: float
    speed: float  # meters per second when moving
    state: str = "idle"  # idle, moving, charging, to_charge, loading, unloading
    path: List[str] = field(default_factory=list)  # sequence of nodes to follow
    path_progress: int = 0  # index in path
    task_id: Optional[str] = None
    total_distance_travelled: float = 0.0
    energy_used: float = 0.0  # percent battery used (for reporting, not physical kWh)
    time_until_free: float = 0.0  # seconds until finishing current action (used for loading/unloading)

# -----------------------------
# World / Graph model
# -----------------------------

class World:
    def __init__(self):
        self.G = nx.Graph()
        self.node_coords = {}
        self.charging_nodes = set()
    
    def add_node(self, name, x, y, charging=False):
        self.G.add_node(name)
        self.node_coords[name] = (x,y)
        if charging:
            self.charging_nodes.add(name)
    
    def add_edge(self, a, b):
        ax, ay = self.node_coords[a]
        bx, by = self.node_coords[b]
        dist = math.hypot(ax-bx, ay-by)
        self.G.add_edge(a,b, length=dist)
    
    def shortest_path(self, a, b):
        return nx.shortest_path(self.G, a, b, weight='length')
    
    def path_length(self, path):
        if not path or len(path) < 2: return 0.0
        s = 0.0
        for i in range(len(path)-1):
            s += self.G[path[i]][path[i+1]]['length']
        return s
    
    def nearest_charging(self, node):
        # find nearest charging node (by shortest path distance)
        best = None; bestd = float('inf')
        for c in self.charging_nodes:
            try:
                p = nx.shortest_path(self.G, node, c, weight='length')
                d = self.path_length(p)
                if d < bestd:
                    bestd = d; best = c
            except nx.NetworkXNoPath:
                continue
        return best, bestd

# -----------------------------
# Central Server with Market-Based Allocator
# -----------------------------

class CentralServer:
    def __init__(self, world: World, robots: Dict[str, Robot]):
        self.world = world
        self.robots = robots
        self.tasks: Dict[str, Task] = {}
        self.reservations = {}  # not fully used in this simple sim, placeholder for future time-window reservations
        # weights for cost function
        self.w_d = 0.6; self.w_B = 0.2; self.w_L = 0.15; self.w_p = 0.05
        self.d_max = 50.0  # used for normalization (tuned to world size)
        self.large_penalty = 1000.0
        self.min_battery_to_start = 20.0  # do not send robot to pickup if battery below this unless charging en route allowed
    
    def submit_task(self, task: Task):
        self.tasks[task.id] = task
    
    def compute_cost(self, robot: Robot, task: Task):
        # compute travel distance: robot -> pickup -> drop
        try:
            path1 = self.world.shortest_path(robot.node, task.pickup)
            path2 = self.world.shortest_path(task.pickup, task.drop)
        except nx.NetworkXNoPath:
            return float('inf')
        d = self.world.path_length(path1) + self.world.path_length(path2)
        D = d / max(self.d_max, 1e-6)
        Bn = 1.0 - robot.battery/100.0  # low battery increases cost
        Ln = robot.load_weight / max(1e-6, robot.max_capacity)
        Pn = 1.0/(1.0 + task.priority)
        cost = self.w_d*D + self.w_B*Bn + self.w_L*Ln + self.w_p*Pn
        if task.weight > (robot.max_capacity - robot.load_weight):
            cost += self.large_penalty
        # If robot battery is too low to even attempt, add penalty
        # Estimate battery needed: assume 0.2% battery per meter (chosen for simulation)
        est_needed = d * 0.2
        if robot.battery - est_needed < 5.0:
            cost += 500.0  # discourage assignment if not enough battery
        return cost
    
    def allocate(self, current_time):
        # allocate waiting tasks using sealed-bid auction simulated centrally
        waiting = [t for t in self.tasks.values() if t.status == "waiting"]
        for t in waiting:
            bids = {}
            for r in self.robots.values():
                # only consider robots that are not charging or to_charge (unless they can handle it)
                if r.state == 'charging':
                    bids[r.id] = float('inf'); continue
                bids[r.id] = self.compute_cost(r, t)
            # pick minimum cost bid
            winner = min(bids, key=bids.get)
            if bids[winner] >= self.large_penalty:
                # no feasible robot at this time
                continue
            # assign
            t.assigned_robot = winner
            t.assign_time = current_time
            t.status = "assigned"
            r = self.robots[winner]
            # set robot path: robot -> pickup -> drop
            p1 = self.world.shortest_path(r.node, t.pickup)
            p2 = self.world.shortest_path(t.pickup, t.drop)
            full = p1 + p2[1:]
            r.path = full
            r.path_progress = 0
            r.task_id = t.id
            r.state = 'moving'
            # reserve nothing for now; simple model
            # print assignment:
            # print(f"[{current_time:.1f}] Assigned task {t.id} to {winner}; path len {self.world.path_length(full):.1f}m")
    
    def preempt_for_charging(self, current_time):
        # if any robot battery low while moving, plan charging route (to_charge state)
        for r in self.robots.values():
            if r.state in ['moving']:
                # estimate remaining distance on path
                remaining_path = r.path[r.path_progress:]
                remaining_dist = self.world.path_length(remaining_path)
                est_needed = remaining_dist * 0.2
                if r.battery - est_needed < 10.0:
                    # send to nearest charger instead by modifying path
                    charger, d_to_charge = self.world.nearest_charging(r.node)
                    if charger is None: continue
                    # create path to charger plus after charging should resume or reassign
                    try:
                        path_to_charger = self.world.shortest_path(r.node, charger)
                    except nx.NetworkXNoPath:
                        continue
                    r.path = path_to_charger
                    r.path_progress = 0
                    r.state = 'to_charge'
                    # mark task as waiting again if needed
                    if r.task_id:
                        task = self.tasks[r.task_id]
                        if task.status in ['assigned','waiting']:
                            task.status = 'waiting'
                            task.assigned_robot = None
                            task.assign_time = None
                        r.task_id = None
                    # print(f"[{current_time:.1f}] Robot {r.id} diverting to charger {charger}")
    
    def tick(self, current_time):
        # main periodic server actions
        # allocate tasks first
        self.allocate(current_time)
        # then check for preemptive charging needs
        self.preempt_for_charging(current_time)
    
# -----------------------------
# Simulation utilities
# -----------------------------

def create_sample_world():
    w = World()
    # create a small grid-like warehouse with loops and few charging nodes
    coords = {
        'A':(0,0),'B':(10,0),'C':(20,0),'D':(30,0),
        'E':(0,10),'F':(10,10),'G':(20,10),'H':(30,10),
        'I':(0,20),'J':(10,20),'K':(20,20),'L':(30,20)
    }
    for n,(x,y) in coords.items():
        # make corner nodes charging stations at A and L for this scenario
        charging = n in ['A','L']
        w.add_node(n,x,y,charging=charging)
    edges = [
        ('A','B'),('B','C'),('C','D'),
        ('E','F'),('F','G'),('G','H'),
        ('I','J'),('J','K'),('K','L'),
        ('A','E'),('E','I'),
        ('B','F'),('F','J'),
        ('C','G'),('G','K'),
        ('D','H'),('H','L')
    ]
    for a,b in edges: w.add_edge(a,b)
    return w

def create_robots(world):
    robots = {}
    # 4 robots at different starting nodes, heterogeneous capacities and speeds
    robots['r1'] = Robot(id='r1', node='A', pos_xy=world.node_coords['A'], battery=100.0, load_weight=0.0, max_capacity=2.0, speed=0.6)
    robots['r2'] = Robot(id='r2', node='D', pos_xy=world.node_coords['D'], battery=70.0, load_weight=0.0, max_capacity=1.5, speed=0.7)
    robots['r3'] = Robot(id='r3', node='I', pos_xy=world.node_coords['I'], battery=40.0, load_weight=0.0, max_capacity=3.0, speed=0.5)
    robots['r4'] = Robot(id='r4', node='L', pos_xy=world.node_coords['L'], battery=50.0, load_weight=0.0, max_capacity=1.0, speed=0.8)
    return robots

def generate_tasks(world, current_time, count=6):
    # create some tasks with varied weights and priorities
    nodes = list(world.G.nodes())
    tasks = []
    for i in range(count):
        p = random.choice(nodes)
        d = random.choice(nodes)
        while d == p:
            d = random.choice(nodes)
        weight = round(random.choice([0.3,0.5,1.0,1.5,2.0]),2)
        priority = random.choice([1,2,3])
        t = Task(id=f"t{int(current_time)}_{i}", pickup=p, drop=d, weight=weight, priority=priority, request_time=current_time)
        tasks.append(t)
    return tasks

# -----------------------------
# Low-level robot motion and battery model
# -----------------------------

def move_robot_along_path(robot: Robot, world: World, dt: float):
    # if robot has no path, remain idle
    if not robot.path or robot.path_progress >= len(robot.path)-1:
        # if ending at charging node and state==to_charge -> arrive and start charging
        if robot.state in ['to_charge'] and robot.node in world.charging_nodes:
            robot.state = 'charging'
            robot.time_until_free = 0.0
        # if task unloading/loading time left, handle
        return
    # movement along current edge between nodes
    cur = robot.path[robot.path_progress]
    nxt = robot.path[robot.path_progress+1]
    edge_len = world.G[cur][nxt]['length']
    # progress tracked as fractional distance along current edge
    if not hasattr(robot, 'edge_progress'):
        robot.edge_progress = 0.0
    travel = robot.speed * dt
    remaining_edge = edge_len - robot.edge_progress
    if travel < remaining_edge:
        # advance along edge
        robot.edge_progress += travel
        robot.total_distance_travelled += travel
        # battery consumption: assume 0.2% per meter (scaled)
        robot.battery = max(0.0, robot.battery - travel * 0.2)
        robot.energy_used += travel * 0.2
    else:
        # finish this edge and move to next node
        robot.total_distance_travelled += remaining_edge
        robot.battery = max(0.0, robot.battery - remaining_edge * 0.2)
        robot.energy_used += remaining_edge * 0.2
        robot.node = nxt
        robot.pos_xy = world.node_coords[nxt]
        robot.path_progress += 1
        robot.edge_progress = 0.0
        # check if reached pickup or drop or charger
        # actual pickup/drop handling will be managed by simulation loop
    # if battery hits zero -> robot becomes disabled (for simplicity we'll stop it)
    if robot.battery <= 0.0:
        robot.state = 'disabled'

def charging_step(robot: Robot, dt: float):
    # simple linear charging: 1% battery per second when at charger (fast for simulation)
    if robot.state == 'charging' and robot.node in world.charging_nodes:
        robot.battery = min(100.0, robot.battery + dt * 1.0)
        if robot.battery >= 95.0:
            robot.state = 'idle'

# -----------------------------
# Simulation engine
# -----------------------------

def run_simulation(sim_time=300.0, dt=1.0, task_spawn_times=None):
    # create world and robots
    world = create_sample_world()
    robots = create_robots(world)
    server = CentralServer(world, robots)
    
    # prepare tasks schedule if not provided: spawn tasks at given times
    if task_spawn_times is None:
        task_spawn_times = [5, 10, 20, 35, 50, 70]
    pending_task_list = []
    logs = []
    all_tasks = {}
    
    current_time = 0.0
    spawn_index = 0
    # generate initial pool of tasks
    while current_time <= sim_time:
        # spawn tasks at scheduled times
        if spawn_index < len(task_spawn_times) and current_time >= task_spawn_times[spawn_index]:
            new_ts = generate_tasks(world, current_time, count=2)
            for t in new_ts:
                server.submit_task(t)
                all_tasks[t.id] = t
                logs.append({'time':current_time, 'event':'task_submitted', 'task':t.id, 'pickup':t.pickup, 'drop':t.drop, 'weight':t.weight, 'priority':t.priority})
            spawn_index += 1
        
        # server tick: allocation and charging preemption check
        server.tick(current_time)
        
        # move robots and simulate actions
        for r in robots.values():
            if r.state in ['moving','to_charge']:
                move_robot_along_path(r, world, dt)
                # check if arrived at pickup node and has a task
                if r.task_id:
                    task = server.tasks.get(r.task_id)
                    if task and task.status == 'assigned':
                        # if reached pickup node
                        if r.node == task.pickup and task.pickup_time is None:
                            task.pickup_time = current_time
                            task.status = 'picked'
                            r.state = 'loading'
                            r.time_until_free = 3.0  # seconds to pick up
                            logs.append({'time':current_time, 'event':'pickup_start', 'task':task.id, 'robot':r.id})
                        # after pickup, continue along path to drop
                # if robot heading to charge and reached charger
                if r.state == 'to_charge' and r.node in world.charging_nodes:
                    r.state = 'charging'
                    logs.append({'time':current_time, 'event':'arrived_charger', 'robot':r.id, 'node': r.node})
            elif r.state == 'loading':
                # decrement loading time
                r.time_until_free -= dt
                if r.time_until_free <= 0.0:
                    # finish loading; set load, continue moving to drop
                    # assign weight from task if known
                    if r.task_id:
                        task = server.tasks.get(r.task_id)
                        if task:
                            r.load_weight += task.weight
                            logs.append({'time':current_time, 'event':'pickup_complete', 'task':task.id, 'robot':r.id})
                    r.state = 'moving'
            elif r.state == 'unloading':
                r.time_until_free -= dt
                if r.time_until_free <= 0.0:
                    # finish unloading
                    if r.task_id:
                        task = server.tasks.get(r.task_id)
                        if task:
                            task.drop_time = current_time
                            task.status = 'delivered'
                            logs.append({'time':current_time, 'event':'drop_complete', 'task':task.id, 'robot':r.id})
                            # release load
                            r.load_weight = max(0.0, r.load_weight - task.weight)
                            r.task_id = None
                    r.state = 'idle'
            elif r.state == 'charging':
                charging_step(r, dt)
                if r.state == 'idle':
                    logs.append({'time':current_time, 'event':'charging_complete', 'robot':r.id, 'battery':r.battery})
            elif r.state == 'idle':
                # if robot has a path but is idle, start moving
                if r.path and r.path_progress < len(r.path)-1:
                    r.state = 'moving'
                # otherwise remain idle
                pass
        
        # After movement updates, check for reached drop-offs (if at drop node and task was picked)
        for t in list(server.tasks.values()):
            if t.status == 'picked':
                r = None
                if t.assigned_robot:
                    r = robots.get(t.assigned_robot)
                else:
                    # could be re-assigned later
                    continue
                if r and r.node == t.drop:
                    # begin unloading
                    t.status = 'delivering'
                    r.state = 'unloading'
                    r.time_until_free = 3.0
                    logs.append({'time':current_time, 'event':'drop_start', 'task':t.id, 'robot':r.id})
        
        # simple rule: if robot idle and battery low, send to charge
        for r in robots.values():
            if r.state == 'idle' and r.battery < 30.0:
                charger, d = world.nearest_charging(r.node)
                if charger and charger != r.node:
                    p = world.shortest_path(r.node, charger)
                    r.path = p; r.path_progress = 0; r.state = 'to_charge'
                    logs.append({'time':current_time, 'event':'to_charge', 'robot':r.id, 'target':charger})
        
        # increment time
        current_time += dt
        # break early if all tasks delivered and no more scheduled spawns
        all_done = all((t.status == 'delivered' for t in server.tasks.values())) and spawn_index >= len(task_spawn_times)
        if all_done and current_time > max(task_spawn_times)+5:
            break
    
    # Simulation finished, prepare metrics
    logs_df = pd.DataFrame(logs)
    tasks_df = pd.DataFrame([{
        'task': t.id,
        'pickup': t.pickup,
        'drop': t.drop,
        'weight': t.weight,
        'priority': t.priority,
        'request_time': t.request_time,
        'assign_time': t.assign_time,
        'pickup_time': t.pickup_time,
        'drop_time': t.drop_time,
        'status': t.status,
        'assigned_robot': t.assigned_robot
    } for t in server.tasks.values()])
    
    robots_df = pd.DataFrame([{
        'robot': r.id,
        'start_node': create_robots(world)[r.id].node,
        'end_node': r.node,
        'battery_end': r.battery,
        'distance': r.total_distance_travelled,
        'energy_used_percent': r.energy_used,
        'tasks_done': sum(1 for t in server.tasks.values() if t.assigned_robot==r.id and t.status=='delivered')
    } for r in robots.values()])
    
    # metrics
    delivered = tasks_df[tasks_df['status']=='delivered']
    avg_completion = (delivered['drop_time'] - delivered['request_time']).dropna()
    avg_completion_time = avg_completion.mean() if len(avg_completion)>0 else None
    delivery_success = len(delivered)/len(tasks_df) if len(tasks_df)>0 else 0.0
    
    metrics = {
        'sim_time': current_time,
        'num_tasks': len(tasks_df),
        'delivered': len(delivered),
        'delivery_success_rate': delivery_success,
        'avg_completion_time': avg_completion_time
    }
    
    return {
        'logs': logs_df,
        'tasks': tasks_df,
        'robots': robots_df,
        'metrics': metrics
    }


# Run a single simulation and present results
result = run_simulation(sim_time=200.0, dt=1.0)
print("Simulation finished. Metrics:")
print(json.dumps(result['metrics'], default=str, indent=2))

# Display tables to user
# Display the tasks dataframe (first few rows or full)
print("=== Simulation Tasks ===")
display(result['tasks'])     # or print(result['tasks'].head())

# Display the robots summary
print("=== Robots Summary ===")
display(result['robots'])    # or print(result['robots'])

# Show the first 80 log entries
print("=== Logs (first 80 rows) ===")
print(result['logs'].head(80))

