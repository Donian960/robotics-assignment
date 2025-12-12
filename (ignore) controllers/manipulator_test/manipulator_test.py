"""
Khepera4 Pick & Place:
Camera for detecting & centering the red object.
Lidar for distance alignment ONLY (target = 0.12 m).
"""

from controller import Robot, Keyboard
import numpy as np

# ---------------- CONFIG -----------------
TIME_STEP = 64
MAX_SPEED = 9.42

SEARCH_SPIN_SPEED   = 1.5
ALIGN_ROT_SPEED     = 2.0
DIST_ALIGN_SPEED    = 3.0
GRIPPER_SPEED       = 2.0

TARGET_PICK_DIST    = 0.12      # meters
DIST_TOLERANCE      = 0.01
ALIGN_TOLERANCE     = 0.05

TARGET_COLOR_BGR    = np.array([0,0,255])
COLOR_TOLERANCE     = 85
MIN_OBJECT_AREA     = 30
# ------------------------------------------


class KheperaPP:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep()) or TIME_STEP

        # Keys
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # Motors
        self.left_motor  = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.stop()

        # Gripper
        self.arm  = self.robot.getDevice("horizontal_motor")
        self.fingers = self.robot.getDevice("finger_motor::left")
        self.arm.setVelocity(GRIPPER_SPEED)
        self.fingers.setVelocity(GRIPPER_SPEED)
        self.reset_gripper()

        # Camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.cam_width  = self.camera.getWidth()
        self.cam_height = self.camera.getHeight()

        # Lidar
        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        print("\n--- READY ---")
        print("Press 1 to Pick, 2 to Place\n")

    # -------------------------------------
    # Basic controls
    # -------------------------------------
    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def set_speeds(self, L, R):
        self.left_motor.setVelocity(max(min(L, MAX_SPEED), -MAX_SPEED))
        self.right_motor.setVelocity(max(min(R, MAX_SPEED), -MAX_SPEED))

    def wait(self, sec):
        steps = int((sec*1000)/self.timestep)
        for _ in range(steps):
            self.robot.step(self.timestep)

    def reset_gripper(self):
        self.arm.setPosition(0.0)
        self.fingers.setPosition(0.0)

    # -------------------------------------
    # Camera: find horizontal offset
    # -------------------------------------
    def get_camera_offset(self):
        img = self.camera.getImage()
        if img is None:
            return None

        arr = np.frombuffer(img, np.uint8).reshape((self.cam_height, self.cam_width, 4))
        bgr = arr[:, :, :3]

        diff = np.abs(bgr.astype(int) - TARGET_COLOR_BGR)
        mask = np.all(diff < COLOR_TOLERANCE, axis=2)

        ys, xs = np.where(mask)
        if xs.size < MIN_OBJECT_AREA:
            return None

        cx = np.mean(xs)
        offset = - (cx - self.cam_width/2) / (self.cam_width/2)
        return offset

    # -------------------------------------
    # Scan environment to find object
    # -------------------------------------
    def scan_for_object(self):
        # Forward look
        for _ in range(10):
            if self.robot.step(self.timestep) == -1: return False
            if self.get_camera_offset() is not None:
                print("Object seen forward.")
                return True

        # Left spin
        self.set_speeds(SEARCH_SPIN_SPEED, -SEARCH_SPIN_SPEED)
        for _ in range(40):
            if self.robot.step(self.timestep) == -1: return False
            if self.get_camera_offset() is not None:
                self.stop()
                print("Object seen left.")
                return True
        self.stop()

        # Right spin
        self.set_speeds(-SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED)
        for _ in range(80):
            if self.robot.step(self.timestep) == -1: return False
            if self.get_camera_offset() is not None:
                self.stop()
                print("Object seen right.")
                return True
        self.stop()

        print("Object NOT found.")
        return False

    # -------------------------------------
    # Center object using camera
    # -------------------------------------
    def center_object(self):
        print("Centering object...")

        while self.robot.step(self.timestep) != -1:
            offset = self.get_camera_offset()
            if offset is None:
                print("Lost object while centering!")
                self.stop()
                return False

            print(f"Camera offset: {offset:.3f}")

            if abs(offset) < ALIGN_TOLERANCE:
                print("Object centered.")
                self.stop()
                return True

            if offset > 0:
                self.set_speeds(-ALIGN_ROT_SPEED, ALIGN_ROT_SPEED)
            else:
                self.set_speeds(ALIGN_ROT_SPEED, -ALIGN_ROT_SPEED)

        return False

    # -------------------------------------
    # LIDAR distance alignment
    # -------------------------------------
    def get_lidar_distance(self):
        ranges = self.lidar.getRangeImage()
        if not ranges: return None
        return float(min(ranges))  # closest object

    def align_distance(self):
        print("Aligning distance using LIDAR...")

        for _ in range(200):
            if self.robot.step(self.timestep) == -1:
                return False

            # must keep object in camera
            if self.get_camera_offset() is None:
                print("Lost object during distance alignment!")
                return False

            dist = self.get_lidar_distance()
            if dist is None:
                continue

            d_err = dist - TARGET_PICK_DIST
            print(f"Distance = {dist:.3f}  (err = {d_err:.3f})")

            if abs(d_err) < DIST_TOLERANCE:
                print("Distance aligned!")
                self.stop()
                return True

            speed = d_err * 12.0
            speed = max(min(speed, DIST_ALIGN_SPEED), -DIST_ALIGN_SPEED)
            self.set_speeds(speed, speed)

        print("Distance alignment failed (timeout).")
        return False

    # -------------------------------------
    # Pick Sequence
    # -------------------------------------
    def pick(self):
        print("\n=== PICK START ===")

        if not self.scan_for_object(): return
        if not self.center_object(): return
        if not self.align_distance(): return

        print("Executing PICK...")

        self.fingers.setPosition(0.0)     # open fingers
        self.wait(0.4)

        self.arm.setPosition(-3.0)        # extend/ lower
        self.wait(1.5)

        self.fingers.setPosition(0.42)    # close fingers
        self.wait(1.0)

        self.arm.setPosition(0.0)         # lift
        self.wait(1.5)

        print("PICK COMPLETE.\n")

    # -------------------------------------
    # Place
    # -------------------------------------
    def place(self):
        print("\n=== PLACE START ===")

        self.arm.setPosition(-3.0)
        self.wait(1.2)

        self.fingers.setPosition(0.0)
        self.wait(0.8)

        self.arm.setPosition(0.0)
        self.wait(1.0)

        self.set_speeds(-3.0, -3.0)
        self.wait(0.8)
        self.stop()

        print("PLACE COMPLETE.\n")

    # -------------------------------------
    # Main Loop
    # -------------------------------------
    def run(self):
        last_key = -1
        while self.robot.step(self.timestep) != -1:
            key = self.keyboard.getKey()
            if key != last_key:
                if key == ord('1'):
                    self.pick()
                elif key == ord('2'):
                    self.place()
                last_key = key


# Run controller
if __name__ == "__main__":
    KheperaPP().run()
