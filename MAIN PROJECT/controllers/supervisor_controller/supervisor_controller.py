from controller import Supervisor
import json
import math
import networkx as nx
import heapq
import re
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Tuple

#webots initialization
sup = Supervisor()
timestep = int(sup.getBasicTimeStep())
receiver = sup.getDevice("receiver")
emitter  = sup.getDevice("emitter")
receiver.enable(timestep)

#Ccnfiguration constants to be used throughout the supervisor code
GRID_CELL_SIZE = 1.0
SNAP_THRESHOLD = 0.1       
SAFE_BATTERY_THRESHOLD = 10.0  
BATTERY_DRAIN_IDLE = 0.05      
BATTERY_DRAIN_MOVE = 0.25      
AVG_ROBOT_SPEED = 0.15 
CHARGE_RATE = 5.0              
AVG_MISSION_DISTANCE = 15.0    
WEIGHT_DRAIN_FACTOR = 0.2      # 20% extra drain per kg of weight
CRITICAL_BATTERY_RESERVE = 15.0
DEFAULT_SPEED = 0.15          
HANDLING_OVERHEAD = 5.0       
SPEED_LEARNING_ALPHA = 0.3    # How fast we adapt to new speeds

# Map Data
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

#Helper Functions

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
    # generates F, L, R, U instructions from path and starting orientation
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

current_sim_time = sup.getTime()
# learning constants
LEARNING_INTERVAL = 5.0       # Update learned parameters every 5 seconds
LEARNING_RATE_ALPHA = 0.3     # How fast we adapt (0.1 = slow/stable, 0.5 = fast/reactive)
DEFAULT_IDLE_DRAIN = 0.05
DEFAULT_MOVE_DRAIN = 0.25

known_robots = {
    "Khepera IV": {
        "id": "Khepera IV", "node": "2,2", "x": 2, "y": 2, 
        "orientation": 90, "battery": 100.0, "max_capacity": 5.0, 
        "state": "idle", "current_task": None, "last_update": current_sim_time,
        "initialized": False,  
        "battery_history": [],        # list of (time, battery, state)
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
        "id": "Khepera IV-1", "node": "8,3", "x": 8, "y": 3, 
        "orientation": 90, "battery": 25.0, "max_capacity": 5.0, 
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
    },
    "Norman": {
        "id": "Norman", "node": "0,6", "x": 0, "y": 6, 
        "orientation": 270, "battery": 25.0, "max_capacity": 5.0, 
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
    },
    "Henry": {
        "id": "Henry", "node": "0,1", "x": 0, "y": 1, 
        "orientation": 90, "battery": 25.0, "max_capacity": 5.0, 
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
    },
    "Thomas": {
        "id": "Thomas", "node": "4,4", "x": 4, "y": 4, 
        "orientation": 0, "battery": 25.0, "max_capacity": 5.0, 
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
    },
    "Percy": {
        "id": "Percy", "node": "4,8", "x": 4, "y": 8, 
        "orientation": 270, "battery": 25.0, "max_capacity": 5.0, 
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

# the key logic functions

#Simulates battery drain AND charging physics
def update_batteries():
    now = sup.getTime()
    for rid, r in known_robots.items():
        dt = now - r["last_update"]
        if dt > 1.0: dt = 1.0 
        
        # 1.check if robot is physically at a charger
        clean_node = r["node"].replace("[","").replace("]","").replace(" ","")
        
        if clean_node in CHARGERS:
            if r["state"] in ["idle", "charging", "moving_to_charge"]:
                
                #if just arrived, send charging start command
                if r["state"] == "moving_to_charge":
                    r["state"] = "charging"
                    
                    # send explicit charging command to robot
                    msg = {
                        "type": "charging_start",
                        "robot_id": rid,
                        "charger_location": clean_node,
                        "timestamp": now
                    }
                    emitter.send(json.dumps(msg).encode('utf-8'))
                
                # charge the battery
                r["battery"] = min(100.0, r["battery"] + (CHARGE_RATE * dt))
                
                # if fully charged, mark as idle and send resume command
                if r["battery"] >= 100.0 and r["state"] == "charging":
                    r["state"] = "idle"
                    
                    # send charging complete message
                    msg = {
                        "type": "charging_complete",
                        "robot_id": rid,
                        "final_battery": r["battery"],
                        "timestamp": now
                    }
                    emitter.send(json.dumps(msg).encode('utf-8'))
        else:
            # standard battery drain logic
            if r["state"] != "idle":
                rate = r.get("sim_move_rate", BATTERY_DRAIN_MOVE)
            else:
                rate = r.get("sim_idle_rate", BATTERY_DRAIN_IDLE)
                
            drain = rate * dt
            r["battery"] = max(0.0, r["battery"] - drain)
            
        r["last_update"] = now

# checks idle robots. If battery is below their PERSONAL safe threshold,
# sends them to the nearest charger.
def monitor_charging_needs():
    for rid, r in known_robots.items():
        if not r.get("initialized", False): 
            continue
        # only interrupt IDLE robots
        if r["state"] != "idle": 
            continue
            
        # intellegent threshold calculation
        # How much energy does THIS specific robot need to survive a trip?
        # Use learned rate if available, else default
        my_efficiency = r.get("learned_move_rate", BATTERY_DRAIN_MOVE)
        
        # calculate dynamic buffer
        # Buffer = (Energy to drive avg mission) + (Energy to drive to charger) + Safety
        # We estimate "Energy to drive to charger" as approx 10 meters for safety
        energy_buffer = ((AVG_MISSION_DISTANCE + 10.0) / AVG_ROBOT_SPEED) * my_efficiency
        
        # the threshold to trigger charging
        # Example: If inefficient robot needs 30% for a mission, trigger at 40% (30 + 10 safe)
        trigger_threshold = energy_buffer + SAFE_BATTERY_THRESHOLD
        
        # Cap threshold
        trigger_threshold = min(trigger_threshold, 90.0)

        if r["battery"] < trigger_threshold:
            send_to_charger(r)

def send_to_charger(robot):
    """
    Finds nearest charger and issues a move command.
    Sets robot state to 'moving_to_charge' so it doesn't get assigned tasks.
    """
    start_node = robot["node"]
    
    # find nearest charger
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
        return

    # generate instructions to charger
    path_to_charger = nx.shortest_path(G, start_node, best_charger)
    instr = generate_instructions(path_to_charger, robot["orientation"])
    
    # change robot state
    robot["state"] = "moving_to_charge" 
    robot["current_task"] = None
    robot["current_pickup"] = None
    robot["current_drop"] = best_charger
    
    # send command
    msg = {
        "type": "assign",
        "task_id": "CHARGE_REQ", # Dummy ID
        "robot_id": robot["id"],
        "instructions": instr,
        "pickup_node": start_node,
        "drop_node": best_charger,
        "location": parse_xy(start_node), # Use integer list [x,y]
        "timestamp": sup.getTime()
    }
    emitter.send(json.dumps(msg).encode('utf-8'))

def update_robot_learning(robot):
    """
    Analyzes recent history to estimate specific discharge rates
    for IDLE and MOVING states using Linear Regression logic.
    """
    history = robot["battery_history"]
    if len(history) < 2: return

    # we need to calculate slope: (change in battery) / (change in time)
    
    idle_slopes = []
    move_slopes = []
    
    for i in range(1, len(history)):
        prev = history[i-1]
        curr = history[i]
        
        dt = curr[0] - prev[0]
        db = prev[1] - curr[1] # positive value as it indicates drop in battery
        state = prev[2]        # state during this interval
        
        if dt <= 0.001: continue
        
        drain_rate = db / dt
        
        # filter out unrealistic values
        if 0 <= drain_rate < 5.0: 
            if state == "idle":
                idle_slopes.append(drain_rate)
            else: # busy_loaded or busy_unloaded
                move_slopes.append(drain_rate)

    # exponential moving average: New_Estimate = Alpha * Observed + (1 - Alpha) * Old_Estimate
    
    if idle_slopes:
        avg_observed_idle = sum(idle_slopes) / len(idle_slopes)
        robot["learned_idle_rate"] = (LEARNING_RATE_ALPHA * avg_observed_idle) + \
                                     ((1 - LEARNING_RATE_ALPHA) * robot["learned_idle_rate"])
                                     
    if move_slopes:
        avg_observed_move = sum(move_slopes) / len(move_slopes)
        robot["learned_move_rate"] = (LEARNING_RATE_ALPHA * avg_observed_move) + \
                                     ((1 - LEARNING_RATE_ALPHA) * robot["learned_move_rate"])

    # Clear history to save memory
    robot["battery_history"] = [history[-1]] 
    
#Advanced Utility Function.
# Calculates cost in 'Seconds of Effort', factoring in:
# 1. Full Trip Physics (Robot->Pick->Drop->Charger)
# 2. Payload Weight Impact on Battery
# 3. Aging/Wait Time (Dynamic Priority)
# 4. Battery Scarcity Risk
def check_feasibility_and_cost(robot, task):
    # feasibility checks
    if task.weight > robot["max_capacity"]: return float('inf')
    if robot.get("node") is None: return float('inf')

    try:
        # path calculations in three parts
        # Leg 1: Robot -> Pickup
        path_pickup = nx.shortest_path(G, robot["node"], task.pickup, weight='length')
        dist_pickup = get_path_length(G, path_pickup)
        
        # Leg 2: Pickup -> Drop
        path_drop = nx.shortest_path(G, task.pickup, task.drop, weight='length')
        dist_drop = get_path_length(G, path_drop)
        
        # Leg 3: Drop -> Any Charger (The "Return Ticket")
        # we do this as after dropoff robot should be able to reach SOME charger
        dist_return = float('inf')
        for charger in CHARGERS:
            try:
                p = nx.shortest_path(G, task.drop, charger, weight='length')
                d = get_path_length(G, p)
                if d < dist_return: dist_return = d
            except nx.NetworkXNoPath: continue
            
        if dist_return == float('inf'): return float('inf')

        # energy simulation
        
        base_drain = robot.get("learned_move_rate", BATTERY_DRAIN_MOVE)
        
        # heavy tasks drain battery faster during the carry leg
        loaded_drain = base_drain * (1.0 + (task.weight * WEIGHT_DRAIN_FACTOR))
        
        # Calculate Energy Consumed per Leg
        speed = robot.get("learned_speed", AVG_ROBOT_SPEED)
        
        energy_leg1 = (dist_pickup / speed) * base_drain    # Empty
        energy_leg2 = (dist_drop / speed) * loaded_drain    # Loaded
        energy_leg3 = (dist_return / speed) * base_drain    # Empty (battery)
        
        total_energy_required = energy_leg1 + energy_leg2 + energy_leg3
        
        predicted_end_battery = robot["battery"] - total_energy_required
        
        # Safety Barrier: Must finish mission + return trip with reserve left
        if predicted_end_battery < CRITICAL_BATTERY_RESERVE:
            return float('inf')
        
        #the three cost components, as told in report
        # time cost
        # we penalize Pickup distance (deadheading) by 1.5x
        time_cost = ((dist_pickup * 1.5) + dist_drop) / speed

        # recovery cost
        # time required to charge back the energy used.
        # this makes inefficient robots "expensive" to hire.
        recovery_cost = (energy_leg1 + energy_leg2) / CHARGE_RATE

        # risk cost
        # if battery is 90%, cost is near 0.
        # if battery is 20%, cost is extreme.
        buffer = predicted_end_battery - CRITICAL_BATTERY_RESERVE
        risk_cost = 20.0 * math.exp(-0.2 * buffer)

        # aging / wait time factor
        # the longer a task waits, the "cheaper" it becomes for robots to accept it.
        current_time = sup.getTime()
        wait_time = max(0.0, current_time - task.request_time)
        
        aging_factor = 1.0 + (wait_time / 30.0)
        
        # priority factor: high priority (1) scales costs down significantly
        priority_factor = max(1, task.priority)
        
        # final score as shown in report
        
        total_physical_cost = time_cost + recovery_cost + risk_cost
        final_bid = total_physical_cost*priority_factor / aging_factor
        
        return final_bid

    except nx.NetworkXNoPath:
        return float('inf')
        
# Global Batch Auction with per-task wait-buffer (priority-based holding).
#     Tasks are only considered for bidding after a short wait window depending on their priority.
def allocate_pending_tasks():
    
    WAIT_STEP = 5.0            # seconds per priority level (priority 2 waits WAIT_STEP, p3 waits 2*WAIT_STEP, ...)

    # identify idle robots
    idle_robots = [r for r in known_robots.values() if r["state"] == "idle" and r.get("initialized", False)]
    if not idle_robots:
        return

    # identify waiting tasks and filter by wait-buffer eligibility
    now = sup.getTime()
    waiting_all = [t for t in tasks.values() if t.status == "waiting"]

    if not waiting_all:
        return

    eligible_tasks = []
    skipped_tasks = []
    for t in waiting_all:
        # ensure priority is at least 1
        p = max(1, int(t.priority) if t.priority is not None else 1)
        wait_needed = (p - 1) * WAIT_STEP
        time_waited = max(0.0, now - t.request_time)
        if time_waited >= wait_needed:
            eligible_tasks.append(t)
        else:
            skipped_tasks.append((t.id, p, time_waited, wait_needed))

    
    if not eligible_tasks:
        # nothing eligible this round
        return

    # generate the bid matrix (pairwise bids as before)
    all_bids = []
    for task in eligible_tasks:
        for robot in idle_robots:
            cost = check_feasibility_and_cost(robot, task)
            if cost != float('inf'):
                all_bids.append({
                    "cost": cost,
                    "task": task,
                    "robot": robot
                })

    if not all_bids:
        return

    # finally determine winners (greedy global by cost)
    all_bids.sort(key=lambda x: x["cost"])

    assigned_task_ids = set()
    assigned_robot_ids = set()

    for bid in all_bids:
        t = bid["task"]
        r = bid["robot"]

        # skip if already taken
        if t.id in assigned_task_ids: 
            continue
        if r["id"] in assigned_robot_ids: 
            continue

        # final additional check: make sure task still waiting (race safety)
        if t.status != "waiting":
            continue

        assign_task(r, t)

        # mark as taken
        assigned_task_ids.add(t.id)
        assigned_robot_ids.add(r["id"])

# Called when a robot finishes a delivery.
# Calculates actual effective speed and updates the learned model.
def finalize_and_learn(robot):
    now = sup.getTime()
    
    #how long since the task started
    total_time_taken = now - robot["task_start_time"]
    
    #subtract handling overhead
    driving_time = total_time_taken - HANDLING_OVERHEAD
    
    # safety check
    if driving_time <= 1.0: 
        return

    # calculate real speed
    dist = robot["task_total_distance"]
    measured_speed = dist / driving_time
    
    # filter anomalies (e.g. robot got stuck and took 10 mins -> speed near 0)
    if 0.05 < measured_speed < 1.0:
        old_speed = robot["learned_speed"]
        
        #exponential moving average update
        new_speed = (SPEED_LEARNING_ALPHA * measured_speed) + \
                    ((1 - SPEED_LEARNING_ALPHA) * old_speed)
                    
        robot["learned_speed"] = new_speed
        
def assign_task(robot, task):
    task.assigned_robot = robot["id"]
    task.status = "assigned"
    
    start_node = robot["node"]
    
    # calculate Distances
    path_pickup = nx.shortest_path(G, start_node, task.pickup, weight='length')
    dist_pickup = get_path_length(G, path_pickup)
    
    path_drop = nx.shortest_path(G, task.pickup, task.drop, weight='length')
    dist_drop = get_path_length(G, path_drop)
    
    total_dist = dist_pickup + dist_drop
    
    # store data for learning later
    robot["task_total_distance"] = total_dist
    robot["task_start_time"] = sup.getTime()
    
    # calculate ETA
    current_speed = robot.get("learned_speed", DEFAULT_SPEED)
    
    est_duration = (total_dist / current_speed) + HANDLING_OVERHEAD
    
    # assign instructions
    instr = generate_instructions(path_pickup, robot["orientation"])
    
    robot["current_task"] = task.id
    robot["state"] = "busy_unloaded"
    robot["current_pickup"] = task.pickup
    robot["current_drop"] = task.drop
    
    #message for robot identification by robot_id
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
    
    # send eta to client
    msg_client = {
        "type": "task_update",
        "request_id": task.id,
        "status": "assigned",
        "assigned_robot": robot["id"],
        "eta_seconds": round(est_duration, 1),
        "avg_speed": round(current_speed, 3)
    }
    emitter.send(json.dumps(msg_client).encode('utf-8'))
    
print("Server Running...")

while sup.step(timestep) != -1:
    if sup.getTime() < 0.1:
        print("[SUPERVISOR] Main loop running.")

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
                        old_node = r["node"]
                        r["node"] = nearest
                        
                        if r["state"] == "moving_to_charge" and nearest == r.get("current_drop"):
                            r["state"] = "charging"
                            r["current_drop"] = None
                            
                        if nearest != old_node and r["state"] not in ["idle", "charging"]:
                            correction_msg = {
                                "type": "location_update",
                                "robot_id": rid,
                                "location": parse_xy(nearest),
                                "orientation": r["orientation"],
                                "timestamp": sup.getTime()
                            }
                            emitter.send(json.dumps(correction_msg).encode('utf-8'))
                            
                    reported_state = msg.get("state")
                    print(f"[SUPERVISOR] Received status from {rid}: state='{reported_state}', time={sup.getTime():.2f}")

                    # handing based on robot state reports
                    if reported_state == "picked_up":
                        pickup_node = r.get("current_pickup")
                        drop_node = r.get("current_drop")
                        task_id = r.get("current_task")

                        if pickup_node and drop_node and task_id:
                            print(f"[{rid}] Pickup confirmed. Calculating path from {r['node']} to {drop_node}...")

                            # plan path from current location to drop-off
                            try:
                                path_drop = nx.shortest_path(G, r["node"], drop_node, weight='length')
                                instr_drop = generate_instructions(path_drop, r["orientation"])

                                # send instructions to robot
                                msg = {
                                    "type": "assign",
                                    "task_id": task_id,
                                    "robot_id": rid,
                                    "instructions": instr_drop,
                                    "pickup_node": pickup_node,
                                    "drop_node": drop_node,
                                    "location": parse_xy(r["node"]),
                                    "timestamp": sup.getTime()
                                }
                                print(f"[SUPERVISOR] SENT ASSIGNMENT to {rid}: Drop path sent (length {len(instr_drop)} instructions).")
                                emitter.send(json.dumps(msg).encode('utf-8'))
                                
                                # update robot state
                                r["state"] = "busy_loaded"
                                tasks[task_id].status = "picked"
                                
                            except nx.NetworkXNoPath:
                                print(f"[SUPERVISOR ERROR] No path found from {r['node']} to {drop_node}!")
                            except Exception as e:
                                print(f"[SUPERVISOR ERROR] Error during drop path calculation/send: {e}")
                        else:
                            print(f"[SUPERVISOR WARNING] Robot {rid} reported picked_up but is missing necessary task data: TaskID='{task_id}', Drop='{drop_node}'.")


                    if reported_state == "idle":
                        
                        # charging related logic
                        if r["state"] == "charging":
                            pass
                        elif r["state"] == "moving_to_charge" and r["node"] == r.get("current_drop"):
                            r["state"] = "charging"
                            r["current_drop"] = None

                        # task completion logic, robot reports idle
                        elif r.get("current_task") and r["node"] == r.get("current_drop"):
                            
                            finalize_and_learn(r)
                            
                            r["state"] = "idle"
                            tid = r.get("current_task")
                            if tid and tid in tasks:
                                tasks[tid].status = "delivered"
                            
                            r["current_task"] = None
                            r["current_pickup"] = None
                            r["current_drop"] = None
                        
                        # at pickup but not yet confirmed loaded
                        elif r.get("current_task") and r["node"] == r.get("current_pickup"):
                            r["state"] = "busy_loaded"
                            if r.get("current_task"): 
                                tasks[r["current_task"]].status = "picked"
                            
                        #  general moving
                        elif reported_state == "moving":
                            pass
            elif mtype == "request":
                # logic for different types of requests
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

            elif mtype == "startup":
                # startup/init message from robot
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
        except Exception as e:
            print(f"[SUPERVISOR CRASH] Error processing msg: {e}")
            continue
    
    debug_status=[]
    for rid, r in known_robots.items():
        debug_status.append(f"{rid}: [{r['x']:.2f}, {r['y']:.2f}] (Node: {r['node']})")
    
    allocate_pending_tasks()
