from controller import Robot

import time

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

## Main ##

state = "TURN"
# "state" can be one of the following values:
## "FOLLOW" - following a black line
## "TURN" - changing direction at a spot
## "STOPPING" - transition from FOLLOW to TURN
## "IDLE" - unmoving

instructions = "FFRRRRLRFRRRLLFLLFLLLS"
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

start_time = time.time()

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
            
        if state == "TURN": # if it is currently in the middle of turning
        
            ci = instructions[current_instruction] # easy access to the current instruction
        
            if ci == "S": # if instruction is to stop, change to the idle state
                state = "IDLE"
                current_instruction += 1
                print(instructions)
                print(time.time() - start_time)
                
            if ci == "F": # if instruction is to go forwards, change to the follow state
                state = "FOLLOW"
                current_instruction += 1
                
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
            
        if state == "IDLE": # if its idle it stays idle
            left_wheel_motor.setVelocity(0)
            right_wheel_motor.setVelocity(0)
        
        ahead = get_position(300) # checks what is coming up
        position = get_position() # checks where it currently is
        
    else: # if there are no more instructions it stays stopped
        left_wheel_motor.setVelocity(0)
        right_wheel_motor.setVelocity(0)
        
   

# Enter here exit cleanup code.
