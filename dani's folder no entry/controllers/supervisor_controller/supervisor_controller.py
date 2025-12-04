# supervisor.py
from controller import Supervisor
import json, time

def now(): return time.time()

def make_assign(robot_id, task_id, instr):
    return {
        "type": "assign",
        "robot_id": robot_id,
        "task_id": task_id,
        "instructions": instr,
        "timestamp": now()
    }


sup = Supervisor()
timestep = int(sup.getBasicTimeStep())
receiver = sup.getDevice("receiver")
emitter  = sup.getDevice("emitter")
if receiver is None:
    print("Supervisor: ERROR - no device named 'receiver'")
if emitter is None:
    print("Supervisor: ERROR - no device named 'emitter'")

receiver.enable(timestep)
print("Supervisor started: listening for client requests...")

request_count = 0
while sup.step(timestep) != -1:
    # process all incoming packets
    while receiver.getQueueLength() > 0:
        raw = receiver.getString()
        receiver.nextPacket()
        try:
            s = raw
            msg = json.loads(s)
        except Exception as e:
            print("Supervisor: failed to decode incoming packet:", e, raw)
            continue

        if msg.get("type") == "request":
            request_count += 1
            req_id = msg.get("request_id")
            client_id = msg.get("robot_id")
            payload = msg.get("payload")
            print(f"Supervisor: Received request {req_id} from {client_id} payload={payload}")

            # Simple hardcoded assignment policy:
            if request_count == 1:
                target = "Khepera IV"
                instr = "FLLRLS"   # hardcoded instructions for worker1
            else:
                target = "Khepera IV-1"
                instr = "FF"   # hardcoded instructions for worker2

            assign_msg = make_assign(target, req_id, instr)
            emitter.send(json.dumps(assign_msg).encode('utf-8'))
            print(f"Supervisor: Sent assign for {req_id} -> {target}: {instr}")

    # small sleep to avoid busy loop; keep stepping consistent with Webots
    # no explicit time.sleep needed beyond step loop
