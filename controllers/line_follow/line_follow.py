from controller import Robot

robot = Robot()
TIME_STEP = 32

left = robot.getDevice("motor.left")
right = robot.getDevice("motor.right")

left.setPosition(float("inf"))
right.setPosition(float("inf"))

gs_left = robot.getDevice("prox.ground.0")
gs_right = robot.getDevice("prox.ground.1")

gs_left.enable(TIME_STEP)
gs_right.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    l = gs_left.getValue()
    r = gs_right.getValue()

    threshold = 1000
    print(l,r)
    if l < threshold and r < threshold:
        left.setVelocity(4)
        right.setVelocity(4)
    elif l < threshold:
        left.setVelocity(2)
        right.setVelocity(4)
    elif r < threshold:
        left.setVelocity(4)
        right.setVelocity(2)
    else:
        left.setVelocity(0)
        right.setVelocity(0)
