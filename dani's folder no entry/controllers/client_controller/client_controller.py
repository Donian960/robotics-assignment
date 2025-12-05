from controller import Robot
import json
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())
MY_ID = robot.getName() or "client"

# devices
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
if receiver:
    receiver.enable(timestep)

print(f"[{MY_ID}] client started. timestep={timestep} ms")

NODE_HASH = {
    "London": "0,0",
    "Berlin": "6,3",
    "Paris": "2,3",
    "Rome": "2,7",
    "Madrid": "2,8",
    "Lisbon": "8,0",
    "Vienna": "8,3",
    "Prague": "7,7",
}


# schedule: list of events with 'time' as seconds offset from start,
# plus request_id, pickup_node, drop_node, weight, priority
schedule = [
    {"time": 1.0, "request_id": "req-01", "pickup": "Paris", "drop": "2,7", "weight": 0.5, "priority": 1},
    {"time": 2.0, "request_id": "req-02", "pickup": "4,2", "drop": "5,1", "weight": 1.0, "priority": 2},
    {"time": 9.0, "request_id": "req-03", "pickup": "2,3", "drop": "2,7", "weight": 0.5, "priority": 1}
]
for req in schedule:
    if req["pickup"] in NODE_HASH:
        req["pickup"] = NODE_HASH[req["pickup"]]
    if req["drop"] in NODE_HASH:
        req["drop"] = NODE_HASH[req["drop"]]
for ev in schedule:
    ev["sent"] = False

start_time = time.time()
end_time = start_time + max(ev["time"] for ev in schedule) + 5.0

def send_request(ev):
    if emitter is None:
        print(f"[{MY_ID}] ERROR: emitter not found")
        return
        
    msg = {
        "type": "request",
        "request_id": ev["request_id"],
        "pickup_node": ev["pickup"],
        "drop_node": ev["drop"],
        "weight": ev["weight"],
        "priority": ev["priority"],
        "timestamp": time.time()
    }
    
    try:
        emitter.send(json.dumps(msg).encode("utf-8"))
        # FIX: Removed msg['payload']
        print(f"[{MY_ID}] sent {ev['request_id']} at +{time.time()-start_time:.2f}s")
    except Exception as e:
        print(f"[{MY_ID}] emitter.send failed for {ev['request_id']}: {e}")
# main loop
while robot.step(timestep) != -1:
    now = time.time()
    elapsed = now - start_time

    # send scheduled events
    for ev in schedule:
        if not ev["sent"] and elapsed >= ev["time"]:
            send_request(ev)
            ev["sent"] = True

    # receive incoming messages
    if receiver:
        while receiver.getQueueLength() > 0:
            raw = receiver.getString()
            receiver.nextPacket()
            try:
                msg = json.loads(raw)
            except Exception:
                msg = {"raw": str(raw)}
            if(msg.get("type")!="location"):
                print(f"[{MY_ID}] received message: {msg}")

    # finish when done
    if now >= end_time and all(ev["sent"] for ev in schedule):
        print(f"[{MY_ID}] finished schedule; exiting.")
        break
