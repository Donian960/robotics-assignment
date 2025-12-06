from controller import Robot

import time
import json
## Defining Map

# this is a dictionary containing the map data
# for every intersection on the map, it lists its neighbour for each direction
# this is used so the robot can trace its current position

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

## Initialisation ## 
robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Get Devices #
ultrasonic_sensors_names = ["left ultrasonic sensor", 
                            "front left ultrasonic sensor", 
                            "front ultrasonic sensor", 
                            "front right ultrasonic sensor", 
                            "right ultrasonic sensor"]
ultrasonic_sensors = {}
for index, name in enumerate(ultrasonic_sensors_names):
    ultrasonic_sensors[name] = robot.getDevice(name)
    ultrasonic_sensors[name].enable(timestep)

infrared_sensors_names = ["rear left infrared sensor", 
                          "left infrared sensor", 
                          "front left infrared sensor", 
                          "front infrared sensor",
                          "front right infrared sensor", 
                          "right infrared sensor", 
                          "rear right infrared sensor", 
                          "rear infrared sensor",
                          "ground left infrared sensor", 
                          "ground front left infrared sensor", 
                          "ground front right infrared sensor",
                          "ground right infrared sensor"]
infrared_sensors = {}
for index, name in enumerate(infrared_sensors_names):
    infrared_sensors[name] = robot.getDevice(name)
    infrared_sensors[name].enable(timestep)
  
led_names = ["front left led",
             "front right led",
             "rear led"]
leds = {}
for name in led_names:
    leds[name] = robot.getDevice(name)

camera = robot.getDevice("camera")
camera.enable(timestep)

emitter = robot.getDevice("emitter")

receiver = robot.getDevice("receiver")
receiver.enable(timestep)

left_wheel_motor = robot.getDevice("left wheel motor");
right_wheel_motor = robot.getDevice("right wheel motor");

left_wheel_motor.setPosition(float('+inf'))
left_wheel_motor.setVelocity(0)

right_wheel_motor.setPosition(float('+inf'))
right_wheel_motor.setVelocity(0)

left_wheel_sensor = robot.getDevice("left wheel sensor")
left_wheel_sensor.enable(timestep)

right_wheel_sensor = robot.getDevice("right wheel sensor")
right_wheel_sensor.enable(timestep)

cam_width = camera.getWidth()
cam_height = camera.getHeight()

def get_position(y = cam_height-2): 
    # gets the current position on the grid based on the colours
    ## y = cam_height - 2 is used to detect the robot's current position
    ## y = 300 is used to detect what is ahead of the robot
        
    t = 0
    r = 0
    g = 0
    b = 0
    
    img = camera.getImage()
    
    for x in range(cam_width // 3, cam_width // 3 * 2): # runs through the centre pixels
        t += 1
        r += camera.imageGetRed(img, cam_width, x, y)
        g += camera.imageGetGreen(img, cam_width, x, y)
        b += camera.imageGetBlue(img, cam_width, x, y)
    
    r = r / t
    g = g / t
    b = b / t
    
    # getting its position based on colour values
    
    if r < 96 and b < 96 and g > 256-96:
        return "green"
    if r > 256-96 and b < 96 and g < 96:
        return "red"
    if r < 96 and b > 256-96 and g < 96:
        return "blue"
    if r < 96 and b < 96 and g < 96:
        return "black"
    return "white"
    
    # possible positions:
    ## "black" - an edge / line
    ## "white" - the vast expanse of nothing
    ## "red" - an intersections
    ## "green" - intended to represent a charging port, but functions like red
    ## "blue" - intended to represent an end point, but functions like red
    
def adjustment():
    # gets which side has more black and so the robot should reallign
    
    l = 0
    r = 0
    
    img = camera.getImage()
    
    for x in range(cam_width // 4, cam_width // 4 * 3): # checks the centre of the x-axis
        if x < cam_width // 2:
            l += camera.imageGetGray(img, cam_width, x, 400)
        if x > cam_width // 2:
            r += camera.imageGetGray(img, cam_width, x, 400)
            
    return l, r
    
def send_status_update(robot_id, loc, orient, state="busy_loaded"):
    msg = {
        "type": "status",
        "robot_id": robot_id,
        "location": loc,
        "orientation": orient,  # <--- NEW FIELD
        "battery": 100.0,
        "state": state,
        "timestamp": time.time()
    }
    emitter.send(json.dumps(msg).encode("utf-8"))
    
def trace(location, direction, turn): 
    # 1. Update Direction
    if turn == "R":
        direction += 90
    elif turn == "L":
        direction -= 90
    elif turn == "U":
        direction += 180
    direction = direction % 360
    
    # 2. Update Location (Safe Mode)
    loc_str = str(location)
    
    # Check if the current location exists and has a path in that direction
    if loc_str in MAP and direction in MAP[loc_str]:
        print(f"Tracing: {location} -> {MAP[loc_str][direction]} (Dir: {direction})")
        return MAP[loc_str][direction], direction
    else:
        # If invalid move (wall/dead end), print warning and STAY PUT
        print(f"WARNING: Wall detected at {location} facing {direction}. Staying put.")
        return location, direction
def wait_for_supervisor_config():
    """
    Blocks execution until initial state is received from Supervisor.
    """
    print(f"[{robot.getName()}] Requesting config from Supervisor...")
    
    # 1. Send the "I am awake" message
    msg = {
        "type": "startup", 
        "robot_id": robot.getName(), 
        "timestamp": time.time()
    }
    emitter.send(json.dumps(msg).encode("utf-8"))
    
    # 2. Wait loop
    while robot.step(timestep) != -1:
        if receiver.getQueueLength() > 0:
            raw = receiver.getString()
            receiver.nextPacket()
            try:
                data = json.loads(raw)
                # Check if this is the response intended for ME
                if data.get("type") == "init" and data.get("robot_id") == robot.getName():
                    
                    loc = data.get("location")
                    ori = data.get("orientation")
                    
                    print(f"[{robot.getName()}] Config received: Loc={loc}, Dir={ori}")
                    return loc, ori
                    
            except json.JSONDecodeError:
                pass
                
    return [0, 0], 90 # Fallback (should not happen)

## Main ##
current_location = [0, 0] 
current_direction = 90
def follow_instructions(instructions,start_loc, start_dir):
    location = start_loc
    direction = start_dir
    state = "TURN"
    # "state" can be one of the following values:
    ## "FOLLOW" - following a black line
    ## "TURN" - changing direction at a spot
    ## "STOPPING" - transition from FOLLOW to TURN
    ## "IDLE" - unmoving
    
    #instructions = "FLLRRFFFLRRS"
    # "instructions" can be made up of the following characters:
    ## "F" - move forwards at a spot
    ## "R" - turn right at a spot
    ## "L" - turn left at a spot
    ## "U" - turn around at a spot (u-turn, treated as being two right turns)
    ## "S" - stop at a spot
    
    current_instruction = 0
    # index of the current instruction in the instructions string
    
    turns = 0
    # used in the "TURN" state
    
    robot.step(timestep)
    ahead = get_position(300)
    position = get_position()
    # getting initial values of ahead and position
    
    #direction = 90
    # direction value used when tracing own steps
    ## represents angle in degrees
    ## 0 is facing right, 90 down, etc.
    
    #location = [0, 0]
    # for tracing own steps, maybe to be sent to HQ?
    ## x, y value from top-left corner
    ## for both direction and location, when i refer to "right, top left, etc." I mean when alligned to the floor's top view
    
    start_time = time.time()
    last_sent_node = None
    while robot.step(timestep) != -1:
    
        if len(instructions) > 0 and current_instruction < len(instructions):
        
            if state == "FOLLOW": # if it is currently following a line
            
                l, r = adjustment() # checks what side of the line it is on
                
                dif = (l - r) / 1000
                
                # speeds are set based on what side of the line it is on, as well as how far off the line it is
                
                if dif >= 0.15:
                
                    # slower speeds are used the further off-line it is so it can readjust easier
            
                    left_wheel_motor.setVelocity(10 + (dif / 2))
                    right_wheel_motor.setVelocity(10 - (dif / 2))
                    
                elif dif < 0.15 and dif >= 0.05:
            
                    left_wheel_motor.setVelocity(15 + (dif / 2))
                    right_wheel_motor.setVelocity(15 - (dif / 2))
                    
                else:
                
                    left_wheel_motor.setVelocity(20 + (dif / 2))
                    right_wheel_motor.setVelocity(20 - (dif / 2))
                
                if ahead != "black" and ahead != "white": # if it detects a spot, swaps to stopping
                    state = "STOPPING"
                
            if state == "STOPPING": # if the robot is stopping at an intersection
            
                if instructions[current_instruction] != "F": # if the robot is not currently stopping
                
                    if position != "black": # if it is fully on the spot, slows down
                    
                        left_wheel_motor.setVelocity(5)
                        right_wheel_motor.setVelocity(5)
                        spot_edge_check = get_position(310)
                        
                        if (spot_edge_check == "black" or spot_edge_check == "white"): # if it sees the far side of the spot, stops and swaps to turn mode
    
                            left_wheel_motor.setVelocity(0)
                            right_wheel_motor.setVelocity(0)
                            turns = 0 
                            state = "TURN"
                            
                    else: # if its still partially on the line, is slower but not too slow
                    
                        left_wheel_motor.setVelocity(10)
                        right_wheel_motor.setVelocity(10)
                        
                else: # if the robot is meant to pass through the intersection
                
                    left_wheel_motor.setVelocity(20)
                    right_wheel_motor.setVelocity(20)
                    
                    if ahead == "black": # if it detects an upcoming line, swaps back to following
                    
                        state = "FOLLOW"
                        current_instruction += 1
                            
                        location, direction = trace(location, direction, "F")
                
            if state == "TURN": # if it is currently in the middle of turning
            
                ci = instructions[current_instruction] # easy access to the current instruction
            
                if ci == "S": # if instruction is to stop, change to the idle state
                    state = "IDLE"
                    current_instruction += 1
                    print(instructions)
                    print(time.time() - start_time)
                    send_status_update(robot.getName(), location, direction, "idle")
                    print(location)
                    
                if ci == "F": # if instruction is to go forwards, change to the follow state
                    state = "FOLLOW"
                    current_instruction += 1
                            
                    location, direction = trace(location, direction, ci)
                    send_status_update(robot.getName(), location, direction, "busy_loaded")
                if ci == "R" or ci == "U": # if instruction is to turn right or uturn, increases motor speed to do so
                
                    left_wheel_motor.setVelocity(10)
                    right_wheel_motor.setVelocity(-10)
                    
                if ci == "L": # if instruction is to turn left, increases motor speed to do so
                
                    left_wheel_motor.setVelocity(-10)
                    right_wheel_motor.setVelocity(10)
                    
                if ci == "R" or ci == "U" or ci == "L": # for any still-turning instruction
                    
                    new_ahead = get_position(300)
                    if new_ahead != ahead and new_ahead == "black": # checks if it has spotted a line
                        turns += 1 # if so then it has turned once
                        
                        if (turns == 1 and (ci == "R" or ci == "L")) or (turns == 2 and ci == "U"): # if its turned far enough
                            left_wheel_motor.setVelocity(0) # then it stops turning
                            right_wheel_motor.setVelocity(0)
                            
                            state = "FOLLOW" # and swaps to follow mode
                            current_instruction += 1
                            
                            location, direction = trace(location, direction, ci)
                            #send_status_update(robot.getName(), location, "busy_loaded")
                            send_status_update(robot.getName(), location, direction, "busy_loaded")
            if state == "IDLE": # if its idle it stays idle
                left_wheel_motor.setVelocity(0)
                right_wheel_motor.setVelocity(0)
            
            ahead = get_position(300) # checks what is coming up
            position = get_position() # checks where it currently is
            
        else: # if there are no more instructions
            left_wheel_motor.setVelocity(0)
            right_wheel_motor.setVelocity(0)
            
            # ... (existing spot check logic) ...
            
            # spot = position 
            # if spot in ("red", "green", "blue") or state == "IDLE":
                #FIXED: Protocol must match Supervisor "status" format
                # node_key = f"{int(location[0])},{int(location[1])}"
                
                # if node_key != last_sent_node:
                    # msg = {
                        # "type": "status",           # CHANGED from "location"
                        # "robot_id": robot.getName(),
                        # "location": location,       # CHANGED to list [x,y]
                        # "battery": 100.0,           # Added dummy battery
                        # "state": "idle",            # Added state
                        # "timestamp": time.time()
                    # }
                    # emitter.send(json.dumps(msg).encode("utf-8"))
                    # last_sent_node = node_key
            
            # FIXED: Break the loop so the robot can hear the next assignment!
            spot = position 
            #if spot in ("red", "green", "blue") or state == "IDLE":
            node_key = f"{int(location[0])},{int(location[1])}"
            
            if node_key != last_sent_node:
                
                # --- INSERT HERE (4/4) ---
                # Final safety update
                send_status_update(robot.getName(), location, direction, "idle")
                
                last_sent_node = node_key
            
            return location, direction
            
print(robot.getName())
current_location, current_direction = wait_for_supervisor_config()
while robot.step(timestep) != -1:
    print(robot.getName(),current_direction)
    if receiver.getQueueLength() > 0:
        msg = receiver.getString()
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            # Convert plain string into an object with instructions
            data = {"instructions": raw}
        print("the message is"+msg)
        if(data.get("type")!="location" and data.get("robot_id")==robot.getName()):
            #print(data)
            instructions = data.get("instructions")
            location=data.get("location")
            print("start following")
            current_location, current_direction = follow_instructions(instructions, current_location, current_direction)
        receiver.nextPacket()

# Enter here exit cleanup code.
