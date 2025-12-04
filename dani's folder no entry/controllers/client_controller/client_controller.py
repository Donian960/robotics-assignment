# client.py
from controller import Robot
import json, time

robot = Robot()
timestep = int(robot.getBasicTimeStep())
MY_ID = robot.getName() or "client1"   # uses robot name in Scene Tree if set
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
if receiver:
    receiver.enable(timestep)

print(f"[{MY_ID}] client started. Will send two requests 5 seconds apart.")

sent = 0
start_time = time.time()
next_send_time = start_time + 1.0  # send first after 1 second
interval = 5.0

def send_request(req_id):
    payload = {"type":"request", "robot_id":MY_ID, "request_id":req_id,
               "payload": {"task":"test"}, "timestamp":time.time()}
    emitter.send(json.dumps(payload).encode('utf-8'))
    print(f"[{MY_ID}] sent request {req_id}")

# main loop: send two requests spaced by interval, then idle and print any incoming assigns
while robot.step(timestep) != -1:
    now = time.time()
    if sent < 2 and now >= next_send_time:
        sent += 1
        send_request(f"req-{sent:02d}")
        next_send_time = now + interval

    # optionally listen for replies/acks (client's receiver)
    if receiver and receiver.getQueueLength() > 0:
        raw = receiver.getString()
        receiver.nextPacket()
        try:
            s = raw
            msg = json.loads(s)
        except Exception:
            msg = {"raw": str(raw)}
        print(f"[{MY_ID}] received message:", msg)

    # stop after some time to keep the test short
    if now - start_time > 20.0:
        print(f"[{MY_ID}] done test run.")
        break
