# controllers/supervisor_controller/supervisor.py
from controller import Supervisor
import json
import time

# Configuration
SEND_PERIOD = 5.0         # seconds between sends (for testing)
TARGET_ROBOT_ID = None    # e.g. "r1" or None for broadcast
TASK_ID = "hardcoded_test_01"

# Message template (hardcoded instruction)
ASSIGN_INSTRUCTIONS = "FRFRLRS"   # change as needed
ETA_SECONDS = 10.0
EXPECTED_ENERGY = 2.0

def make_assign_message(task_id, robot_id, instructions, eta, expected_energy):
    return {
        "type": "assign",
        "task_id": task_id,
        "robot_id": robot_id,           # None means broadcast, robots should filter
        "instructions": instructions,
        "eta_seconds": eta,
        "expected_energy": expected_energy,
        "timestamp": time.time()
    }

def main():
    sup = Supervisor()
    timestep = int(sup.getBasicTimeStep())  # ms
    emitter = sup.getDevice("emitter")
    if emitter is None:
        print("ERROR: emitter not found on supervisor node")
    else:
        try:
            emitter.setChannel(1)
        except Exception:
            pass
        print("Supervisor emitter ready, channel set to 1 (if supported)")
    

    last_send = -SEND_PERIOD
    print("Supervisor started. Broadcasting hardcoded instructions every", SEND_PERIOD, "s")

    while sup.step(timestep) != -1:
        now = time.time()
        # Send periodically
        if now - last_send >= SEND_PERIOD:
            msg = make_assign_message(
                TASK_ID,
                "Khepera IV",
                ASSIGN_INSTRUCTIONS,
                ETA_SECONDS,
                EXPECTED_ENERGY
            )
            try:
                b = json.dumps(msg).encode("utf-8")
                emitter.send(b)
                print(f"[{time.strftime('%H:%M:%S')}] Supervisor: sent assign ->", msg)
            except Exception as e:
                print("Emitter send failed:", e)
            last_send = now

if __name__ == "__main__":
    main()
