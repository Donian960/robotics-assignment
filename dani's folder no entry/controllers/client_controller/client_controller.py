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

# schedule: list of events with 'time' as seconds offset from start,
# plus request_id, pickup_node, drop_node, weight, priority
schedule = [
    {"time": 1.0, "request_id": "req-01", "pickup": "0,6", "drop": "3,4", "weight": 0.5, "priority": 1},
    {"time": 6.0, "request_id": "req-02", "pickup": "1,6", "drop": "3,8", "weight": 1.0, "priority": 2}
]

for ev in schedule:
    ev["sent"] = False

start_time = time.time()
end_time = start_time + max(ev["time"] for ev in schedule) + 5.0

def send_request(ev):
    if emitter is None:
        print(f"[{MY_ID}] ERROR: emitter not found, cannot send {ev['request_id']}")
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
        print(f"[{MY_ID}] sent {ev['request_id']} at +{time.time()-start_time:.2f}s -> {msg['payload']}")
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
            print(f"[{MY_ID}] received message: {msg}")

    # finish when done
    if now >= end_time and all(ev["sent"] for ev in schedule):
        print(f"[{MY_ID}] finished schedule; exiting.")
        break
