from controller import Robot


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
    
    if r < 96 and b < 96 and g > 256-96:
        return "green"
    if r > 256-96 and b < 96 and g < 96:
        return "red"
    if r < 96 and b > 256-96 and g < 96:
        return "blue"
    if r < 96 and b < 96 and g < 96:
        return "black"
    return "white"
    
def adjustment():
    # gets which side has more black and so the robot should reallign
    
    l = 0
    r = 0
    
    img = camera.getImage()
    
    for x in range(cam_width // 4, cam_width // 4 * 3):
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

instructions = "FRLRRLRS"
# "instructions" can be made up of the following characters:
## "F" - move forwards at a spot
## "R" - turn right at a spot
## "L" - turn left at a spot
## "U" - turn around at a spot (u-turn, treated as being two right turns)
## "S" - stop at a spot

current_instruction = 0

turns = 0

robot.step(timestep)
ahead = get_position(300)
position = get_position()

while robot.step(timestep) != -1:
    
    if state == "FOLLOW":
        l, r = adjustment()
        
        l_dif = l - 1035
        r_dif = r - 1035
        
        if r_dif != 0 and l_dif != 0:
    
            left_wheel_motor.setVelocity(8 + (l_dif / 1000))
            right_wheel_motor.setVelocity(8 + (r_dif / 1000))
            
        else:
            left_wheel_motor.setVelocity(10)
            right_wheel_motor.setVelocity(10)
        
        if ahead != "black" and ahead != "white":
            state = "STOPPING"
        
    if state == "STOPPING":
        if position != "black":
            left_wheel_motor.setVelocity(5)
            right_wheel_motor.setVelocity(5)
            spot_edge_check = get_position(310)
            if (spot_edge_check == "black" or spot_edge_check == "white"):
                left_wheel_motor.setVelocity(0)
                right_wheel_motor.setVelocity(0)
                turns = 0 
                state = "TURN"
        else:
            left_wheel_motor.setVelocity(7.5)
            right_wheel_motor.setVelocity(7.5)
        
    if state == "TURN":
    
        ci = instructions[current_instruction] # easy access to the current instruction
    
        if ci == "S": # if instruction is to stop, change to the idle state
            state = "IDLE"
            current_instruction += 1
            
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
            if new_ahead != ahead and new_ahead == "black":
                turns += 1
                
                if (turns == 1 and (ci == "R" or ci == "L")) or (turns == 2 and ci == "U"):
                    left_wheel_motor.setVelocity(0)
                    right_wheel_motor.setVelocity(0)
                    
                    state = "FOLLOW"
                    current_instruction += 1
    
    ahead = get_position(300)
    position = get_position()
   

# Enter here exit cleanup code.
