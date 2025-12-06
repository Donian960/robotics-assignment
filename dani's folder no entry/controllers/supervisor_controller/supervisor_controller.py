from controller import Supervisor
import json
import math
import networkx as nx
import heapq
import re
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Tuple

# --- Webots Initialization (Must be first for time sync) ---
sup = Supervisor()
timestep = int(sup.getBasicTimeStep())
receiver = sup.getDevice("receiver")
emitter  = sup.getDevice("emitter")
receiver.enable(timestep)

# --- Configuration Constants ---
GRID_CELL_SIZE = 1.0
SNAP_THRESHOLD = 0.1       # Metres (Fixed: was 0, needs tolerance)
SAFE_BATTERY_THRESHOLD = 10.0  
BATTERY_DRAIN_IDLE = 0.05      
BATTERY_DRAIN_MOVE = 0.25      
AVG_ROBOT_SPEED = 0.15         

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

# --- Helper Functions ---

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
    
    # FIX: Use inequalities (> 0) instead of (== 1) to handle jumps > 1m
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

# --- State Initialization ---

G, COORDS = build_graph(MAP)
print(f"Graph built with {len(G.nodes)} nodes.")

# Use sup.getTime() for initial timestamps to match Webots simulation time
current_sim_time = sup.getTime()

known_robots = {
    "Khepera IV": {
        "id": "Khepera IV", "node": "0,1", "x": 0, "y": 1, 
        "orientation": 90, "battery": 100.0, "max_capacity": 2.0, 
        "state": "idle", "current_task": None, "last_update": current_sim_time
    },
    "Khepera IV-1": {
        "id": "Khepera IV-1", "node": "8,3", "x": 8, "y": 3, 
        "orientation": 270, "battery": 100.0, "max_capacity": 1.0, 
        "state": "idle", "current_task": None, "last_update": current_sim_time
    }
}

tasks: Dict[str, Task] = {}
pending_heap = []

# --- Logic Functions ---

def update_batteries():
    """Simulates battery drain using simulation time."""
    now = sup.getTime() # FIX: Use simulation time, not wall clock time
    for rid, r in known_robots.items():
        dt = now - r["last_update"]
        if dt > 1.0: dt = 1.0 
        
        rate = BATTERY_DRAIN_MOVE if r["state"] != "idle" else BATTERY_DRAIN_IDLE
        drain = rate * dt
        
        r["battery"] = max(0.0, r["battery"] - drain)
        r["last_update"] = now

def check_feasibility_and_cost(robot, task):
    if task.weight > robot["max_capacity"]: return float('inf')
    if robot.get("node") is None: return float('inf')

    try:
        path_to_pickup = nx.shortest_path(G, robot["node"], task.pickup, weight='length')
        dist_to_pickup = get_path_length(G, path_to_pickup)
        
        path_to_drop = nx.shortest_path(G, task.pickup, task.drop, weight='length')
        dist_to_drop = get_path_length(G, path_to_drop)
        
        min_charge_dist = float('inf')
        for charger in CHARGERS:
            try:
                p = nx.shortest_path(G, task.drop, charger, weight='length')
                d = get_path_length(G, p)
                if d < min_charge_dist: min_charge_dist = d
            except nx.NetworkXNoPath: continue
                
        if min_charge_dist == float('inf'): return float('inf')

        total_mission_dist = dist_to_pickup + dist_to_drop + min_charge_dist
        est_drain = (total_mission_dist / AVG_ROBOT_SPEED) * BATTERY_DRAIN_MOVE
        predicted_battery = robot["battery"] - est_drain
        
        if predicted_battery < SAFE_BATTERY_THRESHOLD:
            return float('inf')
            
        return dist_to_pickup + dist_to_drop

    except nx.NetworkXNoPath:
        return float('inf')

def allocate_pending_tasks():
    repush = []
    while pending_heap:
        prio, rtime, tid = heapq.heappop(pending_heap)
        t = tasks.get(tid)
        
        if not t or t.status != "waiting": continue
        
        best_robot = None
        best_cost = float('inf')
        
        for rid, r in known_robots.items():
            if r["state"] != "idle": continue
            
            cost = check_feasibility_and_cost(r, t)
            if cost < best_cost:
                best_cost = cost
                best_robot = r
        
        if best_robot:
            assign_task(best_robot, t)
        else:
            repush.append((prio, rtime, tid))
            
    for item in repush:
        heapq.heappush(pending_heap, item)

def assign_task(robot, task):
    task.assigned_robot = robot["id"]
    task.status = "assigned"
    
    start_node = robot["node"]
    path_pickup = nx.shortest_path(G, start_node, task.pickup)
    path_drop = nx.shortest_path(G, task.pickup, task.drop)
    
    full_path_nodes = path_pickup + path_drop[1:]
    instr = generate_instructions(full_path_nodes, robot["orientation"])
    
    robot["current_task"] = task.id
    robot["state"] = "busy_unloaded"
    robot["current_pickup"] = task.pickup
    robot["current_drop"] = task.drop
    
    # FIX: Parse start_node to ensure integer location [x, y] in message
    # This aligns with the robot's MAP keys (e.g. "[0, 0]")
    start_x, start_y = parse_xy(start_node)
    
    msg = {
        "type": "assign",
        "task_id": task.id,
        "robot_id": robot["id"],
        "instructions": instr,
        "pickup_node": task.pickup,
        "drop_node": task.drop,
        "location": [start_x, start_y],
        "timestamp": sup.getTime()
    }
    emitter.send(json.dumps(msg).encode('utf-8'))
    print(f"Allocated {task.id} to {robot['id']} (Bat: {robot['battery']:.1f}%)")

# --- Main Loop ---

print("Server Running...")

while sup.step(timestep) != -1:
    update_batteries()
    #print(known_robots)

    while receiver.getQueueLength() > 0:

        try:
            raw = receiver.getString()
            receiver.nextPacket()
            msg = json.loads(raw)
            mtype = msg.get("type")
            #print("message test"+raw)
            if mtype == "status":
                rid = msg.get("robot_id")
                if rid in known_robots:
                    r = known_robots[rid]
                    r["x"] = float(msg.get("location", [0,0])[0])
                    r["y"] = float(msg.get("location", [0,0])[1])
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
                    reported_state = msg.get("state")
                    if reported_state:
                        r["state"] = reported_state
                        if reported_state == "idle" and r.get("current_task"):
                                tid = r.get("current_task")
                                if tid in tasks and tasks[tid].status != "delivered":
                                    tasks[tid].status = "delivered"
                                    r["current_task"] = None
                                    print(f"{rid} forced delivery completion based on IDLE report")
             
            elif mtype == "request":
                rid = msg.get("request_id")
                t = Task(
                    id=rid,
                    pickup=fmt_key(msg.get("pickup_node")),
                    drop=fmt_key(msg.get("drop_node")),
                    weight=msg.get("weight", 0),
                    priority=msg.get("priority", 0)
                )
                tasks[rid] = t
                heapq.heappush(pending_heap, (-t.priority, sup.getTime(), rid))
                print(f"New Request: {rid} ({t.pickup} -> {t.drop})")
            elif mtype == "startup":
                rid = msg.get("robot_id")
                if rid in known_robots:
                    r = known_robots[rid]
                    
                    # Convert internal "0,0" string to [0, 0] list for the robot
                    start_x, start_y = parse_xy(r["node"]) 
                    
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
    print(known_robots)
    allocate_pending_tasks()