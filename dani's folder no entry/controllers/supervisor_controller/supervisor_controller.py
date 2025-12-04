# controllers/supervisor_controller/supervisor_controller.py
"""
Minimal Supervisor Server for request -> assignment.

Features:
- Builds a graph from MAP (string keys like "[0, 6]").
- Listens for 'status' (robot -> server) and 'request' (client -> server).
- Does basic allocation (distance cost) and sends an 'assign' message with path.
- Uses getDevice(...) and receiver.getString() for Webots compatibility.
"""

from controller import Supervisor
import json, time, math
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import networkx as nx
import re


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

CHARGERS = [[0, 0], [6, 3]] # green spots

ENDS = [[2, 3], [2, 7], [2, 8], [8, 0], [8, 3], [7, 7]] # blue spots


# minimal dataclasses used by the supervisor
@dataclass
class Task:
    id: str
    pickup: str
    drop: str
    weight: float
    priority: int
    request_time: float = 0.0
    assigned_robot: Optional[str] = None
    assign_time: Optional[float] = None
    status: str = "waiting"

@dataclass
class RobotState:
    id: str
    node: Optional[str] = None   # current node key like "[0, 6]"
    battery: float = 100.0
    max_capacity: float = 2.0
    state: str = "idle"         # idle, busy

# grid and snap constants — tune to your world
GRID_CELL_SIZE = 1.0     # meters between grid intersections
SNAP_THRESHOLD = 0.4     # meters max distance to snap to an intersection


# helper: turn a "x,y" or other node rep into integer pair
def parse_key_to_xy(k):
    # accepts "x,y", "[x, y]" , (x,y), [x,y]
    if isinstance(k, (list, tuple)):
        return int(k[0]), int(k[1])
    s = str(k)
    nums = re.findall(r"-?\d+", s)
    if len(nums) >= 2:
        return int(nums[0]), int(nums[1])
    raise ValueError("bad node key: " + str(k))

# map vector delta to absolute heading (0=right, 90=down, 180=left, 270=up)
def vector_to_heading(dx, dy):
    # assumes moves between immediate adjacent grid nodes (dx,dy in {-1,0,1} scaled by grid)
    # direction convention: right=(1,0)->0, down=(0,1)->90, left=(-1,0)->180, up=(0,-1)->270
    if dx == 1 and dy == 0:
        return 0
    if dx == 0 and dy == 1:
        return 90
    if dx == -1 and dy == 0:
        return 180
    if dx == 0 and dy == -1:
        return 270
    # if not unit-step, normalize to sign
    sx = 0 if dx == 0 else (1 if dx > 0 else -1)
    sy = 0 if dy == 0 else (1 if dy > 0 else -1)
    if sx == 1 and sy == 0: return 0
    if sx == 0 and sy == 1: return 90
    if sx == -1 and sy == 0: return 180
    if sx == 0 and sy == -1: return 270
    raise ValueError(f"non-axis move dx={dx},dy={dy}")

# normalize angle to 0..359
def norm(a):
    a = int(round(a)) % 360
    return a

# convert delta heading into turn character(s)
def heading_delta_to_instruction(current_h, desired_h):
    d = (desired_h - current_h) % 360
    if d == 0:
        return "F", desired_h
    if d == 90:
        return "R", desired_h
    if d == 270:
        # -90 equivalent to 270 mod 360 -> left turn
        return "L", desired_h
    if d == 180:
        # use U (or "RR" could be used)
        return "U", desired_h
    # For safety, choose the shortest turn: if d==45/135 etc map to nearest cardinal
    # but this shouldn't happen on grid-aligned map
    # fallback:
    if d < 180:
        return "R", desired_h
    else:
        return "L", desired_h

def path_nodes_to_instructions(path, start_heading=None, append_stop=False):
    """
    path: list of node keys in your 'x,y' string format, e.g. ["0,0","0,1","0,6",...]
    start_heading: optional int heading (0,90,180,270). If None, assume facing first segment.
    append_stop: if True, append 'S' at the end.
    returns: instructions string (e.g. "FLLRLS")
    """
    if not path or len(path) < 2:
        return "S" if append_stop else ""
    # parse nodes to integer coords
    coords = [parse_key_to_xy(p) for p in path]
    # compute headings per segment
    seg_headings = []
    for i in range(len(coords) - 1):
        x0, y0 = coords[i]
        x1, y1 = coords[i+1]
        dx = x1 - x0
        dy = y1 - y0
        # if your grid uses inverted y (top-left origin), adapt here accordingly.
        h = vector_to_heading(dx, dy)
        seg_headings.append(h)

    # initial heading
    if start_heading is None:
        # assume facing first segment direction
        current_h = seg_headings[0]
    else:
        current_h = norm(start_heading)

    instructions = []
    for desired_h in seg_headings:
        instr, new_h = heading_delta_to_instruction(current_h, desired_h)
        instructions.append(instr)
        # after performing the turn, robot will traverse the segment and now face desired_h
        current_h = norm(new_h)

    if append_stop:
        instructions.append("S")
    return "".join(instructions)


def snap_location_to_node(x: float, y: float) -> Optional[str]:
    gx = int(round(x / GRID_CELL_SIZE))
    gy = int(round(y / GRID_CELL_SIZE))
    candidate = str([gx, gy])   # matches keys in MAP
    # distance from continuous pose to candidate node center
    dx = x - (gx * GRID_CELL_SIZE)
    dy = y - (gy * GRID_CELL_SIZE)
    dist = math.hypot(dx, dy)
    if dist <= SNAP_THRESHOLD and candidate in MAP:
        return candidate
    return None

# fallback: nearest node by coords (COORDS should map node_key -> (mx,my))
def nearest_node_from_xy(x: float, y: float):
    best = None; bd = float('inf')
    for key, (cx, cy) in COORDS.items():
        d = math.hypot(x - cx, y - cy)
        if d < bd:
            bd = d; best = key
    return best, bd

def node_from_xy(x, y):
    key = f"[{int(x)}, {int(y)}]"
    return key if key in MAP else None

def node_to_key_str(n):
    """
    Normalize any form of coordinate (tuple/list/string) into 'x,y' string.
    """
    if isinstance(n, (list, tuple)):
        return f"{int(n[0])},{int(n[1])}"

    s = str(n)
    nums = re.findall(r"-?\d+", s)
    if len(nums) >= 2:
        return f"{int(nums[0])},{int(nums[1])}"

    raise ValueError(f"Cannot parse node: {n}")


def build_graph_from_map(MAP):
    """
    NEW SIMPLE VERSION.
    Builds a graph with node keys always as 'x,y' strings.
    Computes edge length automatically.
    Returns: G, COORDS
    """
    G = nx.Graph()
    COORDS = {}

    # Add nodes
    for raw_key in MAP.keys():
        key = node_to_key_str(raw_key)

        # Extract numbers for coords
        nums = re.findall(r"-?\d+", str(raw_key))
        x, y = int(nums[0]), int(nums[1])
        COORDS[key] = (float(x), float(y))

        G.add_node(key)

    # Add edges
    for raw_key, neighbours in MAP.items():
        key = node_to_key_str(raw_key)

        for _, nb in neighbours.items():
            nb_key = node_to_key_str(nb)

            # If neighbouring node not added yet, add it with fresh coords
            if nb_key not in COORDS:
                nums = re.findall(r"-?\d+", str(nb))
                nx_, ny_ = int(nums[0]), int(nums[1])
                COORDS[nb_key] = (float(nx_), float(ny_))
                G.add_node(nb_key)

            # Add edge if absent
            if not G.has_edge(key, nb_key):
                (ax, ay) = COORDS[key]
                (bx, by) = COORDS[nb_key]
                dist = math.hypot(ax - bx, ay - by)
                G.add_edge(key, nb_key, length=dist)

    return G, COORDS

def path_length(G, path):
    if not path or len(path) < 2:
        return 0.0
    s = 0.0
    for i in range(len(path)-1):
        s += G[path[i]][path[i+1]]['length']
    return s
# small helper for path length (keeps your old semantics)
def path_length(G: nx.Graph, path: list) -> float:
    if not path or len(path) < 2:
        return 0.0
    return sum(G[path[i]][path[i+1]]['length'] for i in range(len(path)-1))

# def path_length(G, path):
    # if not path or len(path) < 2: return 0.0
    # return sum(G[path[i]][path[i+1]]['length'] for i in range(len(path)-1))

# prepare world graph once
G, COORDS = build_graph_from_map(MAP)
print("nodes:", len(G.nodes))
# canonicalize incoming pickup/drop before using in shortest_path:

# Known robots (initialize with no node, will be updated on status messages)
# supervisor: simple robot store (no dataclass)
known_robots = {
    "Khepera IV": {
        "id": "Khepera IV",
        "node": "0,0",
        "x": 0,          # initial continuous pose (meters)
        "y": 0,
        "battery": 100.0,
        "orientation":90,
        "max_capacity": 2.0,
        "state": "idle"    # idle or busy
    },
    "Khepera IV-1": {
        "id": "Khepera IV-1",
        "node": "3,2",
        "x": 3,
        "y": 2,
        "orientation":90,
        "battery": 100.0,
        "max_capacity": 1.0,
        "state": "idle"
    }
}

# task store
tasks: Dict[str, Task] = {}

# --- Webots supervisor runtime ---
sup = Supervisor()
timestep = int(sup.getBasicTimeStep())
receiver = sup.getDevice("receiver")
emitter  = sup.getDevice("emitter")
receiver.enable(timestep)

print("Supervisor (server) started, graph nodes:", list(G.nodes))

def compute_cost(robot_dict, task):
    # robot_dict is e.g. known_robots["Khepera IV"]
    if task.weight > robot_dict["max_capacity"]:
        return float('inf')
    if robot_dict.get("node") is None:
        # heavy penalty if robot's node unknown (prefer known-position robots)
        return float('inf')
    try:
        p1 = nx.shortest_path(G, robot_dict["node"], task.pickup, weight='length')
        p2 = nx.shortest_path(G, task.pickup, task.drop, weight='length')
    except nx.NetworkXNoPath:
        return float('inf')
    return path_length(G, p1) + path_length(G, p2)

def allocate_task(task):
    best = None; best_cost = float('inf')
    for r in known_robots.values():
        if r["state"] != "idle":
            continue
        c = compute_cost(r, task)
        if c < best_cost:
            best_cost = c; best = r
    if best is None or best_cost == float('inf'):
        return None
    # assign
    task.assigned_robot = best["id"]
    task.assign_time = time.time()
    task.status = 'assigned'
    best["state"] = 'busy'
    # compute path
    p1 = nx.shortest_path(G, best["node"], task.pickup, weight='length')
    p2 = nx.shortest_path(G, task.pickup, task.drop, weight='length')
    path = p1 + p2[1:]
    instr = path_nodes_to_instructions(path, start_heading=best["orientation"], append_stop=True)
    msg = {
        "type": "assign",
        "task_id": task.id,
        "robot_id": best["id"],
        "path": path,
        "instructions":instr,
        "pickup_node": task.pickup,
        "drop_node": task.drop,
        "location":[best["x"],best["y"]],
        "timestamp": time.time()
    }
    emitter.send(json.dumps(msg).encode('utf-8'))
    print(f"[server] assigned {task.id} -> {best['id']} path_len={len(path)}")
    return best["id"]


print("Ready to receive requests and statuses.")

# main loop
while sup.step(timestep) != -1:
    # process incoming messages
    while receiver.getQueueLength() > 0:
        raw = receiver.getString()
        receiver.nextPacket()
        try:
            msg = json.loads(raw)
        except Exception as e:
            print("Supervisor: bad message:", e, raw)
            continue
        mtype = msg.get("type")
        if mtype == "status":
            rid = msg.get("robot_id")
            loc = msg.get("location")    # expects [x, y] in meters
            battery = msg.get("battery")
            state = msg.get("state")
        
            if rid in known_robots:
                r = known_robots[rid]
                if loc and isinstance(loc, (list, tuple)) and len(loc) >= 2:
                    x, y = float(loc[0]), float(loc[1])
                    r["x"], r["y"] = x, y
                    # try snap; if close to grid intersection set node
                    snapped = snap_location_to_node(x, y)
                    if snapped:
                        r["node"] = snapped
                    else:
                        # optional: keep nearest node if within a reasonable threshold
                        nearest, dist = nearest_node_from_xy(x, y)
                        if dist <= (GRID_CELL_SIZE * 0.75):   # tunable
                            r["node"] = nearest
                if battery is not None:
                    r["battery"] = battery
                if state is not None:
                    r["state"] = state
                print(f"[server] status {rid}: loc=({r['x']:.2f},{r['y']:.2f}) node={r['node']} battery={r['battery']} state={r['state']}")
            else:
                print(f"[server] status from unknown robot {rid}")
        
        elif mtype == "request":
            # create Task and try to allocate immediately
            req_id = msg.get("request_id")
            pickup = msg.get("pickup_node")
            drop = msg.get("drop_node")
            weight = msg.get("weight", 0.0)
            priority = msg.get("priority", 0)
            t = Task(id=req_id, pickup=pickup, drop=drop, weight=weight, priority=priority, request_time=time.time())
            tasks[req_id] = t
            print(f"[server] request {req_id} pickup={pickup} drop={drop} weight={weight} priority={priority}")
            
            assigned = allocate_task(t)
            print(known_robots)
            if assigned is None:
                print(f"[server] could not assign {req_id} now (no available robot or path).")
        else:
            print("Supervisor: unknown message type:", mtype)
    # end processing
    # light housekeeping: you can reattempt unassigned tasks periodically (not implemented)
