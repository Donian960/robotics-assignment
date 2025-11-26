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

## Main ##

once = 0

while robot.step(timestep) != -1:

    if once == 0:
        once = 1

# Enter here exit cleanup code.
