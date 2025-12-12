from controller import Supervisor
import json
import math
import networkx as nx
import heapq
import re
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Tuple
import itertools
from collections import defaultdict

# important note
# most of this code is same as suprtvisor_controller.py in main folder
# the key changes are the new functions related to combinatorial task handling
# thus codes comments are only present in those new/changed sections
# no other comments in other sections to avoid redundancy
# the key code new starts at line 500
sup = Supervisor()
timestep = int(sup.getBasicTimeStep())
receiver = sup.getDevice("receiver")
emitter  = sup.getDevice("emitter")
receiver.enable(timestep)

GRID_CELL_SIZE = 1.0
SNAP_THRESHOLD = 0.1       
SAFE_BATTERY_THRESHOLD = 10.0  
BATTERY_DRAIN_IDLE = 0.05      
BATTERY_DRAIN_MOVE = 0.25      
AVG_ROBOT_SPEED = 0.15 
CHARGE_RATE = 5.0              
AVG_MISSION_DISTANCE = 15.0    
WEIGHT_DRAIN_FACTOR = 0.2      
CRITICAL_BATTERY_RESERVE = 15.0
DEFAULT_SPEED = 0.15          
HANDLING_OVERHEAD = 5.0       
SPEED_LEARNING_ALPHA = 0.3    

MAP = {
    "[0, 0]": {90: [0, 1]},
    "[0, 1]": {0: [1, 1], 90: [0, 6], 270: [0, 0]},
    "[0, 6]": {0: [1, 6], 90: [0, 7], 270: [0, 1]},
    "[0, 7]": {0: [1, 7], 90: [0, 8], 270: [0, 6]},
    "[0, 8]": {0: [1, 8], 270: [0, 7]},
    "[1, 0]": {0: [7, 0], 90: [1, 1]},
    "[1, 1]": {0: [3, 1], 180: [0, 1], 270: [1, 0]},
    "[1, 2]": {0: [2, 2], 90: [1, 6]},
    "[1, 6]": {0: [2, 6], 180: [0, 6], 270: [1, 2]},
    "[1, 7]": {90: [1, 8], 180: [0, 7]},
    "[1, 8]": {0: [2, 8], 180: [0, 8], 270: [1, 7]},
    "[2, 2]": {90: [2, 3], 180: [1, 2]},
    "[2, 3]": {270: [2, 2]},
    "[2, 4]": {0: [3, 4], 90: [2, 6]},
    "[2, 6]": {0: [5, 6], 180: [1, 6], 270: [2, 4]},
    "[2, 7]": {0: [3, 7]},
    "[2, 8]": {180: [1, 8]},
    "[3, 1]": {90: [3, 2], 180: [1, 1]},
    "[3, 2]": {0: [4, 2], 270: [3, 1]},
    "[3, 3]": {0: [4, 3], 90: [3, 4]},
    "[3, 4]": {90: [3, 5], 180: [2, 4], 270: [3, 3]},
    "[3, 5]": {0: [5, 5], 270: [3, 4]},
    "[3, 7]": {90: [3, 8], 180: [2, 7]},
    "[3, 8]": {0: [4, 8], 270: [3, 7]},
    "[4, 1]": {0: [5, 1], 90: [4, 2]},
    "[4, 2]": {180: [3, 2], 270: [4, 1]},
    "[4, 3]": {0: [5, 3], 90: [4, 4], 180: [3, 3]},
    "[4, 4]": {0: [5, 4], 270: [4, 3]},
    "[4, 7]": {0: [5, 7], 90: [4, 8]},
    "[4, 8]": {0: [6, 8], 180: [3, 8], 270: [4, 7]},
    "[5, 1]": {90: [5, 2], 180: [4, 1]},
    "[5, 2]": {0: [6, 2], 90: [5, 3], 270: [5, 1]},
    "[5, 3]": {180: [4, 3], 270: [5, 2]},
    "[5, 4]": {0: [6, 4], 90: [5, 5], 180: [4, 4]},
    "[5, 5]": {90: [5, 6], 180: [3, 5], 270: [5, 4]},
    "[5, 6]": {90: [5, 7], 180: [2, 6], 270: [5, 5]},
    "[5, 7]": {180: [4, 7], 270: [5, 6]},
    "[6, 1]": {0: [7, 1], 90: [6, 2]},
    "[6, 2]": {180: [5, 2], 270: [6, 1]},
    "[6, 3]": {90: [6, 4]},
    "[6, 4]": {90: [6, 6], 180: [5, 4], 270: [6, 3]},
    "[6, 6]": {0: [7, 6], 90: [6, 8], 270: [6, 4]},
    "[6, 8]": {0: [8, 8], 180: [4, 8], 270: [6, 6]},
    "[7, 0]": {90: [7, 1], 180: [1, 0]},
    "[7, 1]": {0: [8, 1], 90: [7, 2], 180: [6, 1], 270: [7, 0]},
    "[7, 2]": {0: [8, 2], 90: [7, 4], 270: [7, 1]},
    "[7, 4]": {0: [8, 4], 90: [7, 6], 270: [7, 2]},
    "[7, 6]": {0: [8, 6], 180: [6, 6], 270: [7, 4]},
    "[7, 7]": {0: [8, 7]},
    "[8, 0]": {90: [8, 1]},
    "[8, 1]": {90: [8, 2], 180: [7, 1], 270: [8, 0]},
    "[8, 2]": {180: [7, 2], 270: [8, 1]},
    "[8, 3]": {90: [8, 4]},
    "[8, 4]": {90: [8, 6], 180: [7, 4], 270: [8, 3]},
    "[8, 6]": {90: [8, 7], 180: [7, 6], 270: [8, 4]},
    "[8, 7]": {90: [8, 8], 180: [7, 7], 270: [8, 6]},
    "[8, 8]": {180: [6, 8], 270: [8, 7]}
}

CHARGERS = ["0,0", "6,3"] 

tasks = {}
@dataclass
class Task:
    id: str
    pickup: str
    drop: str
    weight: float
    priority: int
    request_time: float = 0.0
    assigned_robot: Optional[str] = None
    status: str = "waiting"


def fmt_key(k) -> str:
    """Standardizes keys to 'x,y' string format."""
    if isinstance(k, str):
        k = k.replace("[", "").replace("]", "").replace(" ", "")
        return k
    if isinstance(k, (list, tuple)):
        return f"{int(k[0])},{int(k[1])}"
    return str(k)

def parse_xy(k: str) -> Tuple[int, int]:
    """Parses 'x,y' string back to integers."""
    parts = k.split(',')
    return int(parts[0]), int(parts[1])

def build_graph(map_data):
    """Builds NetworkX graph."""
    g = nx.Graph()
    coords = {}
    for key in map_data:
        str_key = fmt_key(key)
        x, y = parse_xy(str_key)
        coords[str_key] = (float(x), float(y))
        g.add_node(str_key)
        
    for key, neighbors in map_data.items():
        u = fmt_key(key)
        for _, dest in neighbors.items():
            v = fmt_key(dest)
            if v not in coords:
                x, y = parse_xy(v)
                coords[v] = (float(x), float(y))
                g.add_node(v)
            if not g.has_edge(u, v):
                dist = math.dist(coords[u], coords[v])
                g.add_edge(u, v, length=dist)
    return g, coords

def get_path_length(g, path):
    if not path or len(path) < 2: return 0.0
    return sum(g[path[i]][path[i+1]]['length'] for i in range(len(path)-1))

def get_heading(current_node, next_node):
    """Calculates heading 0,90,180,270 based on grid movement."""
    cx, cy = parse_xy(current_node)
    nx, ny = parse_xy(next_node)
    dx, dy = nx - cx, ny - cy
    
    if dx > 0: return 0
    if dx < 0: return 180
    if dy > 0: return 90
    if dy < 0: return 270
    return 0 

def generate_instructions(path, start_orient):
    """Generates F, L, R, U instructions."""
    if not path or len(path) < 2: return "S"
    
    instr = []
    curr_h = start_orient
    
    for i in range(len(path) - 1):
        target_h = get_heading(path[i], path[i+1])
        diff = (target_h - curr_h) % 360
        
        if diff == 0: instr.append("F")
        elif diff == 90: instr.append("R")
        elif diff == 270: instr.append("L")
        elif diff == 180: instr.append("U")
        else: instr.append("F") 
        
        curr_h = target_h
        
    instr.append("S")
    return "".join(instr)

def find_nearest_node(x, y, coords):
    """Finds nearest node. Returns (node_key, distance)."""
    best_node = None
    min_dist = float('inf')
    for node, (nx, ny) in coords.items():
        d = math.dist((x, y), (nx, ny))
        if d < min_dist:
            min_dist = d
            best_node = node
    return best_node, min_dist


G, COORDS = build_graph(MAP)
print(f"Graph built with {len(G.nodes)} nodes.")

current_sim_time = sup.getTime()
LEARNING_INTERVAL = 5.0       
LEARNING_RATE_ALPHA = 0.3     
DEFAULT_IDLE_DRAIN = 0.05
DEFAULT_MOVE_DRAIN = 0.25

known_robots = {
    "Khepera IV": {
        "id": "Khepera IV", "node": "8,1", "x": 8, "y": 1, 
        "orientation": 0, "battery": 100.0, "max_capacity": 5.0, 
        "state": "idle", "current_task": None, "last_update": current_sim_time,
        "initialized": False, 
        "battery_history": [],       
        "sim_idle_rate": 0.05, 
        "sim_move_rate": 0.25,
        "learned_idle_rate": DEFAULT_IDLE_DRAIN,
        "learned_move_rate": DEFAULT_MOVE_DRAIN,
        "last_learning_time": current_sim_time,
        "learned_speed": DEFAULT_SPEED, 
        "task_start_time": 0.0,
        "task_total_distance": 0.0,
    },
    "Khepera IV-1": {
        "id": "Khepera IV-1", "node": "2,8", "x": 2, "y": 8, 
        "orientation": 90, "battery": 100.0, "max_capacity": 5.0, 
        "state": "idle", "current_task": None, "last_update": current_sim_time,
        "initialized": False,  
        "battery_history": [],        
        "sim_idle_rate": 0.05, 
        "sim_move_rate": 0.05,
        "learned_idle_rate": DEFAULT_IDLE_DRAIN,
        "learned_move_rate": DEFAULT_MOVE_DRAIN,
        "last_learning_time": current_sim_time,
        "learned_speed": DEFAULT_SPEED,  
        "task_start_time": 0.0,
        "task_total_distance": 0.0,
    }
}

tasks: Dict[str, Task] = {}

def process_assigned_but_idle_robots():
    """
    Safety mechanism: Checks if any idle robot has pending work 
    (queued tasks or stuck current assignments) and forces execution.
    """
    for rid, r in known_robots.items():
        if r["state"] == "idle" and r.get("initialized", False):
            
            stuck_tid = r.get("current_task")
            if stuck_tid:
                if stuck_tid in tasks and tasks[stuck_tid].status != "delivered":
                    print(f"[WATCHDOG] Kickstarting stuck task {stuck_tid} on {rid}")
                    assign_task(r, tasks[stuck_tid])
                    continue 

            queue = r.get("queued_tasks", [])
            if queue and len(queue) > 0:
                next_tid = queue.pop(0)
                
                if next_tid in tasks:
                    print(f"[WATCHDOG] Kickstarting queued task {next_tid} on {rid}")
                    assign_task(r, tasks[next_tid])
                    continue



def update_batteries():
    """Simulates battery drain AND charging physics."""
    now = sup.getTime()
    for rid, r in known_robots.items():
        dt = now - r["last_update"]
        if dt > 1.0: dt = 1.0 
        
        clean_node = r["node"].replace("[","").replace("]","").replace(" ","")
        
        if clean_node in CHARGERS:
            if r["state"] in ["idle", "charging", "moving_to_charge"]:
                
                if r["state"] == "moving_to_charge":
                    r["state"] = "charging"
                    
                    msg = {
                        "type": "charging_start",
                        "robot_id": rid,
                        "charger_location": clean_node,
                        "timestamp": now
                    }
                    emitter.send(json.dumps(msg).encode('utf-8'))
                    print(f"[SUPERVISOR] Sent charging_start command to {rid} at {clean_node}")
                
                r["battery"] = min(100.0, r["battery"] + (CHARGE_RATE * dt))
                
                if r["battery"] >= 100.0 and r["state"] == "charging":
                    r["state"] = "idle"
                    
                    msg = {
                        "type": "charging_complete",
                        "robot_id": rid,
                        "final_battery": r["battery"],
                        "timestamp": now
                    }
                    emitter.send(json.dumps(msg).encode('utf-8'))
                    print(f"[SUPERVISOR] {rid} fully charged ({r['battery']:.1f}%) - sent resume command")
        else:
            if r["state"] != "idle":
                rate = r.get("sim_move_rate", BATTERY_DRAIN_MOVE)
            else:
                rate = r.get("sim_idle_rate", BATTERY_DRAIN_IDLE)
                
            drain = rate * dt
            r["battery"] = max(0.0, r["battery"] - drain)
            
        r["last_update"] = now
        
        
        
        
def monitor_charging_needs():
    """
    Checks idle robots. If battery is below their PERSONAL safe threshold,
    sends them to the nearest charger.
    """
    for rid, r in known_robots.items():
        if not r.get("initialized", False): 
            continue
        if r["state"] != "idle": 
            continue
            
        my_efficiency = r.get("learned_move_rate", BATTERY_DRAIN_MOVE)
        
        energy_buffer = ((AVG_MISSION_DISTANCE + 10.0) / AVG_ROBOT_SPEED) * my_efficiency
        
        trigger_threshold = energy_buffer + SAFE_BATTERY_THRESHOLD
        
        trigger_threshold = min(trigger_threshold, 90.0)

        if r["battery"] < trigger_threshold:
            print(f"[{rid}] Low Battery ({r['battery']:.1f}% < {trigger_threshold:.1f}%). Routing to charger...")
            send_to_charger(r)

def send_to_charger(robot):
    """
    Finds nearest charger and issues a move command.
    Sets robot state to 'moving_to_charge' so it doesn't get assigned tasks.
    """
    start_node = robot["node"]
    
    best_charger = None
    min_dist = float('inf')
    
    for charger in CHARGERS:
        try:
            path = nx.shortest_path(G, start_node, charger, weight='length')
            dist = get_path_length(G, path)
            if dist < min_dist:
                min_dist = dist
                best_charger = charger
        except nx.NetworkXNoPath:
            continue
            
    if best_charger is None:
        print(f"CRITICAL: {robot['id']} cannot reach any charger!")
        return

    path_to_charger = nx.shortest_path(G, start_node, best_charger)
    instr = generate_instructions(path_to_charger, robot["orientation"])
    
    robot["state"] = "moving_to_charge" 
    robot["current_task"] = None
    robot["current_pickup"] = None
    robot["current_drop"] = best_charger 
    
    msg = {
        "type": "assign",
        "task_id": "CHARGE_REQ",
        "robot_id": robot["id"],
        "instructions": instr,
        "pickup_node": start_node,
        "drop_node": best_charger,
        "location": parse_xy(start_node), 
        "timestamp": sup.getTime()
    }
    emitter.send(json.dumps(msg).encode('utf-8'))
    print(f"Sent {robot['id']} to charger at {best_charger}")

def update_robot_learning(robot):
    """
    Analyzes recent history to estimate specific discharge rates
    for IDLE and MOVING states using Linear Regression logic.
    """
    history = robot["battery_history"]
    if len(history) < 2: return

    idle_slopes = []
    move_slopes = []
    
    for i in range(1, len(history)):
        prev = history[i-1]
        curr = history[i]
        
        dt = curr[0] - prev[0]
        db = prev[1] - curr[1] 
        state = prev[2]        
        
        if dt <= 0.001: continue 
        
        drain_rate = db / dt
        
        if 0 <= drain_rate < 5.0: 
            if state == "idle":
                idle_slopes.append(drain_rate)
            else: 
                move_slopes.append(drain_rate)

    if idle_slopes:
        avg_observed_idle = sum(idle_slopes) / len(idle_slopes)
        robot["learned_idle_rate"] = (LEARNING_RATE_ALPHA * avg_observed_idle) + \
                                     ((1 - LEARNING_RATE_ALPHA) * robot["learned_idle_rate"])
                                     
    if move_slopes:
        avg_observed_move = sum(move_slopes) / len(move_slopes)
        robot["learned_move_rate"] = (LEARNING_RATE_ALPHA * avg_observed_move) + \
                                     ((1 - LEARNING_RATE_ALPHA) * robot["learned_move_rate"])

    robot["battery_history"] = [history[-1]] 
    
    print(f"[{robot['id']} Learning] Idle: {robot['learned_idle_rate']:.4f}%/s | Move: {robot['learned_move_rate']:.4f}%/s")

def check_feasibility_and_cost(robot, task):
    """
    Advanced Utility Function.
    Calculates cost in 'Seconds of Effort', factoring in:
    1. Full Trip Physics (Robot->Pick->Drop->Charger)
    2. Payload Weight Impact on Battery
    3. Aging/Wait Time (Dynamic Priority)
    4. Battery Scarcity Risk
    """
    
    if task.weight > robot["max_capacity"]: return float('inf')
    if robot.get("node") is None: return float('inf')

    try:
        
        path_pickup = nx.shortest_path(G, robot["node"], task.pickup, weight='length')
        dist_pickup = get_path_length(G, path_pickup)
        
        path_drop = nx.shortest_path(G, task.pickup, task.drop, weight='length')
        dist_drop = get_path_length(G, path_drop)
        
        dist_return = float('inf')
        for charger in CHARGERS:
            try:
                p = nx.shortest_path(G, task.drop, charger, weight='length')
                d = get_path_length(G, p)
                if d < dist_return: dist_return = d
            except nx.NetworkXNoPath: continue
            
        if dist_return == float('inf'): return float('inf')

        base_drain = robot.get("learned_move_rate", BATTERY_DRAIN_MOVE)
        
        loaded_drain = base_drain * (1.0 + (task.weight * WEIGHT_DRAIN_FACTOR))
        
        speed = robot.get("learned_speed", AVG_ROBOT_SPEED)
        
        energy_leg1 = (dist_pickup / speed) * base_drain    
        energy_leg2 = (dist_drop / speed) * loaded_drain    
        energy_leg3 = (dist_return / speed) * base_drain    
        
        total_energy_required = energy_leg1 + energy_leg2 + energy_leg3
        
        predicted_end_battery = robot["battery"] - total_energy_required
        
        if predicted_end_battery < CRITICAL_BATTERY_RESERVE:
            return float('inf')

        time_cost = ((dist_pickup * 1.5) + dist_drop) / speed

        recovery_cost = (energy_leg1 + energy_leg2) / CHARGE_RATE

        buffer = predicted_end_battery - CRITICAL_BATTERY_RESERVE
        risk_cost = 20.0 * math.exp(-0.2 * buffer)

        current_time = sup.getTime()
        wait_time = max(0.0, current_time - task.request_time)
        
        aging_factor = 1.0 + (wait_time / 30.0)
        
        priority_factor = max(1, task.priority)
        
        total_physical_cost = time_cost + recovery_cost + risk_cost
        final_bid = total_physical_cost * priority_factor /  aging_factor
        
        return final_bid

    except nx.NetworkXNoPath:
        return float('inf')
        
# COMBINATORIAL AUCTION 
# Hyper Parameters
G_MAX_BUNDLE_SIZE = 3    # g : max tasks per robot bundle, large g have huge compute cost
MAX_BUNDLE_COST_THRESHOLD = 500.0 

# precompute pairwise shortest-path distances between all relevant nodes
# we build D[node_u][node_v] = shortest distance (float). Use graph G.
# basically without this precomputation, the combinatorial auction would be too slow
def precompute_distance_matrix(G):
    nodes = list(G.nodes)
    D = {u: {} for u in nodes}
    for u in nodes:
        # single-source Dijkstra for speed and reliability
        lengths = nx.single_source_dijkstra_path_length(G, u, weight='length')
        for v, L in lengths.items():
            D[u][v] = L
    return D

# build D aka the precomputed distance matrix
D = precompute_distance_matrix(G)

#basic travel / energy helpers 
def travel_time(distance, robot_speed):
    if robot_speed <= 0: return float('inf')
    return distance / robot_speed

def energy_for_leg(distance, robot, loaded=False, task_weight=0.0):
    base_drain = robot.get("learned_move_rate", BATTERY_DRAIN_MOVE)
    if loaded:
        drain = base_drain * (1.0 + (task_weight * WEIGHT_DRAIN_FACTOR))
    else:
        drain = base_drain
    speed = robot.get("learned_speed", AVG_ROBOT_SPEED)
    t = travel_time(distance, speed)
    return t * drain

# evaluate a full ordered sequence of tasks for a robot 
def evaluate_sequence(robot, ordered_task_ids):
    """
    For a robot and an ordered list of task ids (order of tasks, interpreted as
    visit pick->drop for each task in that list), compute feasibility (battery reserve
    at each step) and compute final bid (cost) using same economic model as check_feasibility_and_cost.
    """
    if not ordered_task_ids:
        return False, None, None, {"reason": "empty_sequence"}
    current_node = robot.get("node")
    battery = robot["battery"]
    speed = robot.get("learned_speed", AVG_ROBOT_SPEED)

    battery_profile = [battery]
    time_total = 0.0
    energy_used_total = 0.0
    min_buffer = float('inf')

    # accumulate distances for time_cost computation
    deadhead_distance = 0.0
    loaded_distance = 0.0

    for tid in ordered_task_ids:
        if tid not in tasks:
            return False, None, None, {"reason": "task_missing", "task": tid}
        t = tasks[tid]
        # node distances via precomputed D
        d1 = D.get(current_node, {}).get(t.pickup, float('inf'))
        if d1 == float('inf'):
            return False, None, None, {"reason": "no_path_to_pickup", "task": tid}
        e1 = energy_for_leg(d1, robot, loaded=False, task_weight=0.0)
        t1 = travel_time(d1, speed)
        time_total += t1
        energy_used_total += e1
        battery -= e1
        battery_profile.append(battery)
        min_buffer = min(min_buffer, battery - CRITICAL_BATTERY_RESERVE)
        if battery < CRITICAL_BATTERY_RESERVE:
            return False, None, None, {"reason": "battery_below_reserve_on_pick", "task": tid}
        deadhead_distance += d1

        d2 = D.get(t.pickup, {}).get(t.drop, float('inf'))
        if d2 == float('inf'):
            return False, None, None, {"reason": "no_path_pick_to_drop", "task": tid}
        e2 = energy_for_leg(d2, robot, loaded=True, task_weight=t.weight)
        t2 = travel_time(d2, speed)
        time_total += t2
        energy_used_total += e2
        battery -= e2
        battery_profile.append(battery)
        min_buffer = min(min_buffer, battery - CRITICAL_BATTERY_RESERVE)
        if battery < CRITICAL_BATTERY_RESERVE:
            return False, None, None, {"reason": "battery_below_reserve_on_drop", "task": tid}
        loaded_distance += d2

        current_node = t.drop

    # return to nearest charger
    best_dist_charger = float('inf')
    for ch in CHARGERS:
        d = D.get(current_node, {}).get(ch, float('inf'))
        if d < best_dist_charger:
            best_dist_charger = d
    if best_dist_charger == float('inf'):
        return False, None, None, {"reason": "no_path_to_charger"}
    e3 = energy_for_leg(best_dist_charger, robot, loaded=False, task_weight=0.0)
    t3 = travel_time(best_dist_charger, speed)
    time_total += t3
    energy_used_total += e3
    battery -= e3
    battery_profile.append(battery)
    min_buffer = min(min_buffer, battery - CRITICAL_BATTERY_RESERVE)
    if battery < CRITICAL_BATTERY_RESERVE:
        return False, None, None, {"reason": "battery_below_reserve_on_return"}

    # economic cost (same model as the single-task)
    time_cost = ((deadhead_distance * 1.5) + loaded_distance) / speed
    recovery_cost = energy_used_total / CHARGE_RATE
    buffer = min_buffer
    risk_cost = 20.0 * math.exp(-0.2 * buffer)

    # aging & priority aggregation: using average wait time and average priority
    current_time = sup.getTime()
    wait_times = [max(0.0, current_time - tasks[tid].request_time) for tid in ordered_task_ids]
    avg_wait = sum(wait_times) / max(1, len(wait_times))
    aging_factor = 1.0 + (avg_wait / 30.0)
    priorities = [tasks[tid].priority for tid in ordered_task_ids]
    priority_factor = 3.0 / max(1, sum(priorities) / len(priorities))

    total_physical_cost = time_cost + recovery_cost + risk_cost
    final_bid = total_physical_cost / (priority_factor * aging_factor)

    details = {
        "time_total": time_total,
        "energy_used_total": energy_used_total,
        "battery_profile": battery_profile,
        "min_buffer": min_buffer,
        "deadhead": deadhead_distance,
        "loaded": loaded_distance
    }
    return True, final_bid, battery_profile, details

# exact DP for ordering a small subset of tasks (pickup before drop precedence) 
def best_sequence_for_subset(robot, subset_task_ids):
    """
    subset_task_ids: list of task ids (k)
    this DP minimizes travel-time including final leg to charger.
    """
    k = len(subset_task_ids)
    # map node indices: for i in 0..k-1 => pick index = 2*i, drop index = 2*i+1
    node_positions = []
    node_to_task = {}
    for i, tid in enumerate(subset_task_ids):
        t = tasks[tid]
        node_positions.append(t.pickup)   # 2*i
        node_positions.append(t.drop)     # 2*i + 1
        node_to_task[2*i] = ("pick", tid)
        node_to_task[2*i + 1] = ("drop", tid)

    FULL_MASK = (1 << (2*k)) - 1
    # DP map: DP[mask][last_idx] -> min_time
    DP = [defaultdict(lambda: float('inf')) for _ in range(1 << (2*k))]
    PREV = {}

    start_node = robot.get("node")
    # initialize picks
    for pick_idx in range(0, 2*k, 2):
        node_id = node_positions[pick_idx]
        dist = D.get(start_node, {}).get(node_id, float('inf'))
        if dist == float('inf'): continue
        DP[1 << pick_idx][pick_idx] = travel_time(dist, robot.get("learned_speed", AVG_ROBOT_SPEED))
        PREV[(1 << pick_idx, pick_idx)] = (0, -1)

    # iterate masks
    for mask in range(1 << (2*k)):
        for last_idx, cost_so_far in list(DP[mask].items()):
            last_node_graph_id = start_node if last_idx == -1 else node_positions[last_idx]
            for nxt in range(2*k):
                if (mask >> nxt) & 1: 
                    continue
                # If nxt is a drop node ensure its pick visited
                if nxt % 2 == 1:
                    pick_idx = nxt - 1
                    if ((mask >> pick_idx) & 1) == 0:
                        continue
                nxt_node_graph_id = node_positions[nxt]
                dist = D.get(last_node_graph_id, {}).get(nxt_node_graph_id, float('inf'))
                if dist == float('inf'):
                    continue
                ttime = travel_time(dist, robot.get("learned_speed", AVG_ROBOT_SPEED))
                new_mask = mask | (1 << nxt)
                new_cost = cost_so_far + ttime
                if new_cost < DP[new_mask].get(nxt, float('inf')):
                    DP[new_mask][nxt] = new_cost
                    PREV[(new_mask, nxt)] = (mask, last_idx)

    # add final leg to charger
    best_cost = float('inf')
    best_end = None
    for last_idx, cost_so_far in DP[FULL_MASK].items():
        last_node_id = node_positions[last_idx]
        # nearest charger distance
        best_ch_dist = min((D.get(last_node_id, {}).get(ch, float('inf')) for ch in CHARGERS), default=float('inf'))
        if best_ch_dist == float('inf'):
            continue
        total = cost_so_far + travel_time(best_ch_dist, robot.get("learned_speed", AVG_ROBOT_SPEED))
        if total < best_cost:
            best_cost = total
            best_end = last_idx

    if best_end is None:
        return None, float('inf')

    # reconstruct node visit order
    seq_nodes = []
    mask = FULL_MASK
    last = best_end
    while mask:
        seq_nodes.append(last)
        prev_mask, prev_last = PREV[(mask, last)]
        mask, last = prev_mask, prev_last
    seq_nodes.reverse()

    # convert visit nodes to ordered task ids (every two nodes correspond to one task in pick->drop order)
    ordered_task_ids = []
    # Since DP ensured picks before drops, iterate and append task ids when encountering a pick node
    picked = set()
    for node_idx in seq_nodes:
        kind, tid = node_to_task[node_idx]
        if kind == "pick" and tid not in picked:
            ordered_task_ids.append(tid)
            picked.add(tid)
    return ordered_task_ids, best_cost

# generate all feasible bundles for a robot up to size g 
def generate_exact_bundles_for_robot(robot, waiting_task_list, g=G_MAX_BUNDLE_SIZE):
    bundles = []
    N = len(waiting_task_list)
    task_id_list = [t.id for t in waiting_task_list]
    # quick capacity filter
    weight_ok = {t.id: (t.weight <= robot["max_capacity"]) for t in waiting_task_list}

    for k in range(1, min(g, N) + 1):
        for subset in itertools.combinations(task_id_list, k):
            # skip overweight tasks
            if not all(weight_ok[tid] for tid in subset):
                continue

            # cheap lower bound energy/distance prune
            start = robot.get("node")
            min_start_to_pick = min((D.get(start, {}).get(tasks[tid].pickup, float('inf')) for tid in subset), default=float('inf'))
            if min_start_to_pick == float('inf'):
                continue
            # sum pick->drop reachable check
            sum_pd = 0.0
            reachable = True
            for tid in subset:
                d = D.get(tasks[tid].pickup, {}).get(tasks[tid].drop, float('inf'))
                if d == float('inf'):
                    reachable = False
                    break
                sum_pd += d
            if not reachable: continue

            # min drop->charger
            min_drop_to_charger = float('inf')
            for tid in subset:
                for ch in CHARGERS:
                    d = D.get(tasks[tid].drop, {}).get(ch, float('inf'))
                    if d < min_drop_to_charger:
                        min_drop_to_charger = d
            if min_drop_to_charger == float('inf'):
                continue

            # lower-bound energy estimate
            lb_dist = min_start_to_pick + sum_pd + min_drop_to_charger
            # approximate lower bound energy: treat all loaded legs as loaded (conservative)
            lb_energy = energy_for_leg(min_start_to_pick, robot, loaded=False)
            for tid in subset:
                lb_energy += energy_for_leg(D.get(tasks[tid].pickup, {}).get(tasks[tid].drop, 0.0), robot, loaded=True, task_weight=tasks[tid].weight)
            lb_energy += energy_for_leg(min_drop_to_charger, robot, loaded=False)

            if robot["battery"] - lb_energy < CRITICAL_BATTERY_RESERVE:
                continue

            # compute best sequence for this subset
            ordered_task_ids, dp_time_cost = best_sequence_for_subset(robot, list(subset))
            if ordered_task_ids is None:
                continue

            feasible, final_bid, battery_profile, details = evaluate_sequence(robot, ordered_task_ids)
            if not feasible:
                # For tiny k (<=3) brute force permutations as fallback
                if k <= 3:
                    best_fb = None
                    best_bid = float('inf')
                    for perm in itertools.permutations(subset):
                        feas2, bid2, bp2, det2 = evaluate_sequence(robot, list(perm))
                        if feas2 and bid2 < best_bid:
                            best_bid = bid2
                            best_fb = (list(perm), bid2, bp2, det2)
                    if best_fb is None:
                        continue
                    ordered_task_ids, final_bid, battery_profile, details = best_fb
                else:
                    continue

            # add bundle
            bundles.append({
                "robot_id": robot["id"],
                "tasks": ordered_task_ids,
                "cost": final_bid,
                "battery_profile": battery_profile,
                "details": details
            })
    return bundles

# Greedy Winner Determination over Bundles 
def winner_determination_greedy(bundles):
    """
    Select disjoint bundles (each robot at most one bundle, each task at most once).
    Uses cost_per_task = cost / len(tasks) to sort bundles (tends to prefer efficient multi-task bundles).
    Returns list of chosen bundles.
    """
    chosen = []
    used_robots = set()
    used_tasks = set()

    # sort by cost per task ascending (pragmatic). If you want strict min-total-cost selection, use MILP instead.
    bundles_sorted = sorted(bundles, key=lambda b: (b["cost"] / max(1, len(b["tasks"])), b["cost"]))
    for b in bundles_sorted:
        if b["robot_id"] in used_robots:
            continue
        conflict = False
        for tid in b["tasks"]:
            if tid in used_tasks:
                conflict = True
                break
        if conflict:
            continue
        # skip absurdly expensive bundles
        if b["cost"] > MAX_BUNDLE_COST_THRESHOLD:
            continue
        chosen.append(b)
        used_robots.add(b["robot_id"])
        used_tasks.update(b["tasks"])
    return chosen

# New allocate_pending_tasks using bundle generation 
def allocate_pending_tasks():
    idle_robots = [r for r in known_robots.values() if r["state"] == "idle" and r.get("initialized", False)]
    if not idle_robots: 
        return
    waiting_tasks_list = [t for t in tasks.values() if t.status == "waiting"]
    if not waiting_tasks_list:
        return

    # spatial filter per robot to reduce candidate subsets
    SPATIAL_FILTER_RADIUS = 50.0  # ignore tasks whose pickup is far from robot start
    robot_to_local_tasks = {}
    for r in idle_robots:
        local = []
        for t in waiting_tasks_list:
            d = D.get(r["node"], {}).get(t.pickup, float('inf'))
            if d <= SPATIAL_FILTER_RADIUS:
                local.append(t)
        # fallback: if no local tasks, consider all (prevents starvation)
        if not local:
            local = waiting_tasks_list
        robot_to_local_tasks[r["id"]] = local

    # generate bundles in parallel-friendly loop 
    all_bundles = []
    for r in idle_robots:
        local_tasks = robot_to_local_tasks.get(r["id"], waiting_tasks_list)
        b = generate_exact_bundles_for_robot(r, local_tasks, g=G_MAX_BUNDLE_SIZE)
        # annotate actual robot pointer for assignment convenience
        for item in b:
            item["robot_ref"] = r
        all_bundles.extend(b)

    if not all_bundles:
        return

    # winner determination
    winners = winner_determination_greedy(all_bundles)

    # execute assignments for chosen bundles
    for w in winners:
        robot = w["robot_ref"]
        task_ids = w["tasks"]
        # we assign only the first task to the robot's current_task and rely on its internal controller to proceed,
        # For simplicity, we assign tasks sequentially: we send instructions for the first task (pick->drop),
        # and store the remaining tasks in robot state as 'queued_tasks' to be used when it becomes idle again.
        first_tid = task_ids[0]
        if first_tid not in tasks:
            continue
        # mark all tasks in bundle as assigned to this robot (so they are not visible to other bundles)
        for tid in task_ids:
            if tid in tasks:
                tasks[tid].assigned_robot = robot["id"]
                tasks[tid].status = "assigned"
        # attach queued_tasks on robot object for future scheduling
        robot["queued_tasks"] = task_ids[1:]  # remaining tasks to run after first completes
        # assign the first task using existing assign_task
        assign_task(robot, tasks[first_tid])
        print(f"[AUCTION] Robot {robot['id']} assigned bundle {task_ids} (cost {w['cost']:.1f})")



def finalize_and_learn(robot):
    """
    Called when a robot finishes a delivery.
    Calculates actual effective speed and updates the learned model.
    """
    now = sup.getTime()
    
    total_time_taken = now - robot["task_start_time"]
    
    driving_time = total_time_taken - HANDLING_OVERHEAD
    
    if driving_time <= 1.0: 
        return

    dist = robot["task_total_distance"]
    measured_speed = dist / driving_time
    
    if 0.05 < measured_speed < 1.0:
        old_speed = robot["learned_speed"]
        
        new_speed = (SPEED_LEARNING_ALPHA * measured_speed) + \
                    ((1 - SPEED_LEARNING_ALPHA) * old_speed)
                    
        robot["learned_speed"] = new_speed
        
        print(f"[{robot['id']} Learning] Job Done. "
              f"Dist: {dist}m | Time: {total_time_taken:.1f}s | "
              f"Real Speed: {measured_speed:.3f} | New Avg: {new_speed:.3f} m/s")

def assign_task(robot, task):
    task.assigned_robot = robot["id"]
    task.status = "assigned"
    
    start_node = robot["node"]
    
    path_pickup = nx.shortest_path(G, start_node, task.pickup, weight='length')
    dist_pickup = get_path_length(G, path_pickup)
    
    path_drop = nx.shortest_path(G, task.pickup, task.drop, weight='length')
    dist_drop = get_path_length(G, path_drop)
    
    total_dist = dist_pickup + dist_drop
    
    robot["task_total_distance"] = total_dist
    robot["task_start_time"] = sup.getTime()
    
    current_speed = robot.get("learned_speed", DEFAULT_SPEED)
    
    est_duration = (total_dist / current_speed) + HANDLING_OVERHEAD
    
    full_path_nodes = path_pickup + path_drop[1:]
    instr = generate_instructions(full_path_nodes, robot["orientation"])
    
    robot["current_task"] = task.id
    robot["state"] = "busy_unloaded"
    robot["current_pickup"] = task.pickup
    robot["current_drop"] = task.drop
    
    msg = {
        "type": "assign",
        "task_id": task.id,
        "robot_id": robot["id"],
        "instructions": instr,
        "pickup_node": task.pickup,
        "drop_node": task.drop,
        "location": parse_xy(start_node),
        "timestamp": sup.getTime()
    }
    emitter.send(json.dumps(msg).encode('utf-8'))
    
    msg_client = {
        "type": "task_update",
        "request_id": task.id,
        "status": "assigned",
        "assigned_robot": robot["id"],
        "eta_seconds": round(est_duration, 1),
        "avg_speed": round(current_speed, 3)
    }
    emitter.send(json.dumps(msg_client).encode('utf-8'))
    
    print(f"Allocated {task.id}. Dist: {total_dist:.1f}m. ETA: {est_duration:.1f}s")

print("Server Running...")

while sup.step(timestep) != -1:
    update_batteries()
    monitor_charging_needs()
    while receiver.getQueueLength() > 0:

        try:
            raw = receiver.getString()
            receiver.nextPacket()
            msg = json.loads(raw)
            mtype = msg.get("type")
            if mtype == "status":
                rid = msg.get("robot_id")
                if rid in known_robots:
                    r = known_robots[rid]
                    r["x"] = float(msg.get("location", [0,0])[0])
                    r["y"] = float(msg.get("location", [0,0])[1])
                    
                    current_bat = r["battery"]
                    r["battery_history"].append((sup.getTime(), current_bat, r["state"]))
        
                    if sup.getTime() - r["last_learning_time"] > LEARNING_INTERVAL:
                        update_robot_learning(r)
                        r["last_learning_time"] = sup.getTime()
                        
                    new_orient = msg.get("orientation")
                    if new_orient is not None:
                        r["orientation"] = int(new_orient)
                        
                    nearest, dist = find_nearest_node(r["x"], r["y"], COORDS)
                    if dist <= SNAP_THRESHOLD:
                        current_n = r["node"]
                        r["node"] = nearest
                        
                        
                        if r["state"] == "busy_unloaded" and nearest == r.get("current_pickup"):
                            r["state"] = "busy_loaded"
                            if r.get("current_task"): tasks[r["current_task"]].status = "picked"
                            print(f"{rid} arrived at pickup {nearest}")
                            
                        elif r["state"] == "busy_loaded" and nearest == r.get("current_drop"):
                            r["state"] = "idle"
                            tid = r.get("current_task")
                            if tid: tasks[tid].status = "delivered"
                            r["current_task"] = None
                            r["current_pickup"] = None
                            r["current_drop"] = None
                            print(f"{rid} delivered at {nearest}")
                        elif r["state"] == "moving_to_charge" and nearest == r.get("current_drop"):
                            r["state"] = "charging"
                            r["current_drop"] = None
                            print(f"{rid} docked at charger {nearest}. Charging started.")
                        if nearest != old_node and r["state"] not in ["idle", "charging"]:
                            # print(f"[SUPERVISOR] Correcting {rid}: {old_node} → {nearest}")
                            correction_msg = {
                                "type": "location_update",
                                "robot_id": rid,
                                "location": parse_xy(nearest),
                                "orientation": r["orientation"],
                                "timestamp": sup.getTime()
                            }
                            emitter.send(json.dumps(correction_msg).encode('utf-8'))
                    reported_state = msg.get("state")
                    
                    reported_state = msg.get("state")
                    if reported_state == "idle":
                        
                        if r["state"] == "charging":
                            pass
                        elif r["state"] == "moving_to_charge" and r["node"] == r.get("current_drop"):
                            r["state"] = "charging"
                            r["current_drop"] = None
                            print(f"{rid} docked at charger. Charging...")

                        elif r.get("current_task") and r["node"] == r.get("current_drop"):
                            
                            finalize_and_learn(r)
                            tid = r.get("current_task")
                            if tid and tid in tasks:
                                tasks[tid].status = "delivered"
                            
                            print(f"{rid} finished {tid} at {r['node']}.")

                            if r.get("queued_tasks") and len(r["queued_tasks"]) > 0:
                                next_tid = r["queued_tasks"].pop(0) 
                                
                                if next_tid in tasks:
                                    print(f"[{rid}] Chain Active: Autostarting next task {next_tid}")
                                    assign_task(r, tasks[next_tid]) 
                                    
                                else:
                                    r["state"] = "idle"
                                    r["current_task"] = None
                            else:
                                r["state"] = "idle"
                                r["current_task"] = None
                                print(f"{rid} mission complete. Waiting for new orders.")
                        
                        elif r.get("current_task") and r["node"] == r.get("current_pickup"):
                            r["state"] = "busy_loaded"
                            if r.get("current_task"): 
                                tasks[r["current_task"]].status = "picked"
                            
                            print(f"{rid} loaded at Pickup {r['node']}. Continuing to Drop...")
                            
                            
                        elif reported_state == "moving":
                            pass
            elif mtype == "request":
                rid = msg.get("request_id")
                t = Task(
                    id=rid,
                    pickup=fmt_key(msg.get("pickup_node")),
                    drop=fmt_key(msg.get("drop_node")),
                    weight=msg.get("weight", 0),
                    priority=msg.get("priority", 0),
                    request_time=sup.getTime()
                )
                tasks[rid] = t
                print(f"New Request: {rid} ({t.pickup} -> {t.drop})")
            elif mtype == "startup":
                rid = msg.get("robot_id")
                if rid in known_robots:
                    r = known_robots[rid]
                    
                    start_x, start_y = parse_xy(r["node"]) 
                    r["initialized"] = True  
                    init_msg = {
                        "type": "init",
                        "robot_id": rid,
                        "location": [start_x, start_y],
                        "orientation": r["orientation"]
                    }
                    emitter.send(json.dumps(init_msg).encode('utf-8'))
                    print(f"initialized {rid} at {r['node']} facing {r['orientation']}")
        except Exception as e:
            print(f"Error processing msg: {e}")
            continue
  
    allocate_pending_tasks()
    process_assigned_but_idle_robots()
