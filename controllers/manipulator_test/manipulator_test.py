"""
Intelligent Khepera4 + Khepera3 Gripper Pick & Place Controller (FIXED)
- Uses camera vision to locate object
- Scans area when approaching pickup location
- Adjusts position based on visual feedback
- Handles object position uncertainty
"""

from controller import Robot, Motor, GPS, PositionSensor, Camera
import math
import numpy as np

# Constants
MAX_SPEED = 9.42
FWD_SPEED = 8.0
TURN_SPEED = 3.0
GRIPPER_SPEED = 2.0
SCAN_SPEED = 1.5

WHEEL_RADIUS = 0.021
AXLE_LENGTH = 0.1054

# Target locations (approximate)
PICKUP_LOCATION  = [0.5, 0.0]
DROP_LOCATION    = [-0.5, 0.0]

# Vision parameters
SEARCH_DISTANCE = 0.25  
APPROACH_DISTANCE = 0.10  
MIN_OBJECT_AREA = 50  
TARGET_COLOR_BGR = [0, 0, 255] # Blue, Green, Red
COLOR_TOLERANCE = 80  

class Khepera4PickPlace:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Encoders
        self.left_ps = self.robot.getDevice("left wheel sensor")
        self.right_ps = self.robot.getDevice("right wheel sensor")
        self.left_ps.enable(self.timestep)
        self.right_ps.enable(self.timestep)

        # Gripper
        self.arm_motor = self.robot.getDevice("horizontal_motor")
        self.finger_motor = self.robot.getDevice("finger_motor::left")

        # GPS
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        # Camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera_width = self.camera.getWidth()
        self.camera_height = self.camera.getHeight()

        # Heading estimation
        self.prev_left = 0
        self.prev_right = 0
        self.heading = 0.0

        print("Intelligent Pick & Place Controller initialized!")

    # ---------------------- Utility ----------------------------

    def update_heading(self):
        left = self.left_ps.getValue()
        right = self.right_ps.getValue()

        dl = (left - self.prev_left) * WHEEL_RADIUS
        dr = (right - self.prev_right) * WHEEL_RADIUS

        self.prev_left = left
        self.prev_right = right

        dtheta = (dr - dl) / AXLE_LENGTH
        self.heading += dtheta

        return self.heading

    def get_position(self):
        pos = self.gps.getValues()
        return [pos[0], pos[1]]

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def wait(self, sec):
        steps = int((sec * 1000) / self.timestep)
        for _ in range(steps):
            self.robot.step(self.timestep)
            self.update_heading()

    # ---------------------- Vision ----------------------------

    def detect_object(self):
        image = self.camera.getImage()
        if not image:
            return False, 0, 0
        
        img_array = np.frombuffer(image, np.uint8).reshape(
            (self.camera_height, self.camera_width, 4)
        )
        img_bgr = img_array[:, :, :3] 

        # Color Detection
        color_diff = np.abs(img_bgr.astype(int) - TARGET_COLOR_BGR)
        match_mask = np.all(color_diff < COLOR_TOLERANCE, axis=2)
        object_pixels = np.where(match_mask)
        
        if len(object_pixels[0]) < MIN_OBJECT_AREA:
            return False, 0, 0
        
        centroid_x = np.mean(object_pixels[1])
        
        # Calculate offset (-1.0 to 1.0)
        # Positive offset means object is on the RIGHT
        x_offset = (centroid_x - self.camera_width / 2) / (self.camera_width / 2)
        relative_size = len(object_pixels[0]) / (self.camera_width * self.camera_height)
        
        print(f"Object detected! Offset: {x_offset:.2f}, Size: {relative_size:.4f}")
        return True, x_offset, relative_size

    def scan_for_object(self):
        print("Scanning for object...")
        
        # Try center first
        self.robot.step(self.timestep)
        detected, x_offset, size = self.detect_object()
        if detected and abs(x_offset) < 0.3:
            print("Object found at center")
            return True, 0
        
        # Scan left
        print("Scanning left...")
        initial_heading = self.heading
        self.left_motor.setVelocity(-SCAN_SPEED)
        self.right_motor.setVelocity(SCAN_SPEED)
        
        for _ in range(int(0.8 / (self.timestep / 1000.0))): 
            self.robot.step(self.timestep)
            self.update_heading()
            detected, x_offset, size = self.detect_object()
            
            if detected:
                self.stop()
                angle_turned = self.heading - initial_heading
                # Adjust target angle by the visual offset to center it perfectly
                # Note: Field of View scaling factor approx 0.5 for camera to angle mapping
                return True, angle_turned - (x_offset * 0.5) 
        
        self.stop()
        self.wait(0.2)
        
        # Return to center and scan right
        print("Returning to center and scanning right...")
        self.turn_to_angle(initial_heading)
        
        self.left_motor.setVelocity(SCAN_SPEED)
        self.right_motor.setVelocity(-SCAN_SPEED)
        
        for _ in range(int(0.8 / (self.timestep / 1000.0))): 
            self.robot.step(self.timestep)
            self.update_heading()
            detected, x_offset, size = self.detect_object()
            
            if detected:
                self.stop()
                angle_turned = self.heading - initial_heading
                return True, angle_turned - (x_offset * 0.5)
        
        self.stop()
        self.turn_to_angle(initial_heading)
        print("Object not found in scan")
        return False, 0

    def align_with_object(self):
        """
        Fine-tune alignment with object using visual feedback.
        """
        print("Aligning with object...")
        
        for attempt in range(15):
            self.robot.step(self.timestep)
            self.update_heading()
            detected, x_offset, size = self.detect_object()
            
            if not detected:
                print("Lost sight of object during alignment")
                return False
            
            if abs(x_offset) < 0.05: # Strict alignment
                print("Aligned with object!")
                self.stop()
                return True
            
            # --- FIX: Inverted Logic Here ---
            # If x_offset is POSITIVE (Right), we must turn RIGHT.
            # Turn Right: Left Motor (+), Right Motor (-)
            turn_amount = x_offset * 0.8
            turn_amount = max(min(turn_amount, SCAN_SPEED), -SCAN_SPEED)
            
            self.left_motor.setVelocity(turn_amount)   # Changed from negative
            self.right_motor.setVelocity(-turn_amount) # Changed from positive
            
            self.wait(0.1)
            self.stop()
        
        print("Alignment completed (max attempts reached)")
        return True

    def approach_object_visually(self):
        print("Approaching object with visual feedback...")
        
        approach_size_threshold = 0.015 
        
        while self.robot.step(self.timestep) != -1:
            self.update_heading()
            detected, x_offset, size = self.detect_object()
            
            if not detected:
                print("Lost object during approach")
                self.stop()
                return False
            
            if size > approach_size_threshold:
                self.stop()
                print(f"Reached approach distance (size: {size:.4f})")
                return True
            
            # --- FIX: Inverted Approach Logic ---
            if abs(x_offset) > 0.1:
                # Turn in place if offset is too large
                turn_correction = x_offset * 2.0
                turn_correction = max(min(turn_correction, TURN_SPEED), -TURN_SPEED)
                # Turn towards object (Right if offset > 0)
                self.left_motor.setVelocity(turn_correction)
                self.right_motor.setVelocity(-turn_correction)
                self.wait(0.1)
            else:
                # Move forward with curve
                base_speed = 2.0
                # If offset is positive (Right), Left wheel needs to be faster
                left_speed = base_speed + (x_offset * 2.0)
                right_speed = base_speed - (x_offset * 2.0)
                
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                self.wait(0.1)
        
        return False

    # ---------------------- Gripper ----------------------------

    def open_gripper_fingers(self):
        self.finger_motor.setVelocity(GRIPPER_SPEED)
        self.finger_motor.setPosition(0.0)
        self.wait(1.0)

    def close_gripper_fingers(self):
        self.finger_motor.setVelocity(GRIPPER_SPEED)
        self.finger_motor.setPosition(0.42)
        self.wait(1.0)

    def open_gripper_arms(self):
        self.arm_motor.setVelocity(GRIPPER_SPEED)
        self.arm_motor.setPosition(-3.0)
        self.wait(1.0)

    def close_gripper_arms(self):
        self.arm_motor.setVelocity(GRIPPER_SPEED)
        self.arm_motor.setPosition(0.0)
        self.wait(1.0)

    # ---------------------- Navigation ----------------------------

    def turn_to_angle(self, target_angle):
        while self.robot.step(self.timestep) != -1:
            heading = self.update_heading()
            error = target_angle - heading
            error = (error + math.pi) % (2 * math.pi) - math.pi

            if abs(error) < 0.05:
                self.stop()
                break

            sign = 1 if error > 0 else -1
            self.left_motor.setVelocity(-sign * TURN_SPEED)
            self.right_motor.setVelocity(sign * TURN_SPEED)

    def navigate_to(self, tx, ty, stop_distance=0.05):
        print(f"Navigating to: {tx:.2f}, {ty:.2f}")

        while self.robot.step(self.timestep) != -1:
            pos = self.get_position()
            heading = self.update_heading()

            dx = tx - pos[0]
            dy = ty - pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < stop_distance:
                self.stop()
                print("Reached target area.")
                break

            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - heading
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_error) > 0.15:
                turn = 2 * angle_error
                turn = max(min(turn, TURN_SPEED), -TURN_SPEED)
                self.left_motor.setVelocity(-turn)
                self.right_motor.setVelocity(turn)
            else:
                speed = min(FWD_SPEED, dist * 5)
                self.left_motor.setVelocity(speed)
                self.right_motor.setVelocity(speed)

    # ---------------------- Pick & Place ----------------------------

    def pick_object(self):
        print("\n=== INTELLIGENT PICK SEQUENCE ===")

        self.navigate_to(PICKUP_LOCATION[0], PICKUP_LOCATION[1], stop_distance=SEARCH_DISTANCE)
        self.wait(0.5)
        
        found, angle_offset = self.scan_for_object()
        
        if found:
            # Turn to face object
            target_heading = self.heading + angle_offset
            self.turn_to_angle(target_heading)
            self.wait(0.5)
            
            if self.align_with_object():
                self.approach_object_visually()
            else:
                # Blind approach fallback
                self.left_motor.setVelocity(2)
                self.right_motor.setVelocity(2)
                self.wait(0.5)
                self.stop()
        else:
            print("Object not found, using GPS fallback")
            self.navigate_to(PICKUP_LOCATION[0] - 0.15, PICKUP_LOCATION[1])

        print("Executing grasp...")
        self.open_gripper_fingers()
        self.open_gripper_arms()

        self.left_motor.setVelocity(1.5)
        self.right_motor.setVelocity(1.5)
        self.wait(0.7) # Move closer to ensure grip
        self.stop()

        self.close_gripper_fingers()
        self.wait(0.5)
        self.close_gripper_arms()

        self.left_motor.setVelocity(-2)
        self.right_motor.setVelocity(-2)
        self.wait(0.5)
        self.stop()
        
        print("Object picked successfully!")

    def place_object(self):
        print("\n=== PLACE SEQUENCE ===")
        self.navigate_to(DROP_LOCATION[0] + 0.15, DROP_LOCATION[1])

        self.open_gripper_arms()
        
        self.left_motor.setVelocity(2)
        self.right_motor.setVelocity(2)
        self.wait(0.4)
        self.stop()

        self.open_gripper_fingers()
        self.wait(0.5)
        self.close_gripper_arms()

        self.left_motor.setVelocity(-2)
        self.right_motor.setVelocity(-2)
        self.wait(1.0)
        self.stop()
        
        print("Object placed successfully!")

    def run(self):
        print("\n" + "="*60)
        print("STARTING INTELLIGENT PICK & PLACE")
        print("="*60 + "\n")
        self.wait(1)

        self.pick_object()
        self.place_object()

        print("\n" + "="*60)
        print("PICK & PLACE COMPLETED!")
        print("="*60 + "\n")

        while self.robot.step(self.timestep) != -1:
            self.update_heading()

if __name__ == "__main__":
    controller = Khepera4PickPlace()
    controller.run()