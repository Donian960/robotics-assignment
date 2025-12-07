from controller import Robot
from odometry import Odometry
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

left_motor = robot.getDevice("left wheel motor");
right_motor = robot.getDevice("right wheel motor");

left_motor.setPosition(float('+inf'))
left_motor.setVelocity(3)

right_motor.setPosition(float('+inf'))
right_motor.setVelocity(3)

left_sensor = robot.getDevice("left wheel sensor")
left_sensor.enable(timestep)

right_sensor = robot.getDevice("right wheel sensor")
right_sensor.enable(timestep)

gripper_motors = []
gripper_motors.append(robot.getDevice("horizontal_motor"))
gripper_motors.append(robot.getDevice("finger_motor::left"))

name = robot.getName()
time_between_broadcasts = 1000

## Main ##
time_since_last_broadcast = 0
x = 0
y = 0

od = Odometry(0, 0, 0, left_sensor.getValue(), right_sensor.getValue()) 
while robot.step(timestep) != -1:
    
    od.step(left_sensor.getValue(), right_sensor.getValue())
    
    time_since_last_broadcast += timestep
    if time_since_last_broadcast > time_between_broadcasts:
        message = { "name":name,
                    "x": x,
                    "y": y}
        emitter.send(str(message))
        time_since_last_broadcast = 0
    
    pass

# Enter here exit cleanup code.
