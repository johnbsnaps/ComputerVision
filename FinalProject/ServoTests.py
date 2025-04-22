import time
import keyboard
from FinalProject.maestro import Controller

# Maestro channel assignments (update as needed)
STRAIGHT = 0
ROTATE = 1
PAN = 3
TILT = 4

# Movement values (adjust these)
NEUTRAL = 6000
FORWARD = 5200
BACKWARD = 7000
SPIN_RIGHT = 5200
SPIN_LEFT = 6850
PAN_CENTER = 6000
TILT_CENTER = 4400

# Initialize Maestro
maestro = Controller()
maestro.setTarget(STRAIGHT, NEUTRAL)
maestro.setTarget(ROTATE, NEUTRAL)
maestro.setTarget(PAN, PAN_CENTER)
maestro.setTarget(TILT, TILT_CENTER)

# Arm servo channels (placeholders)
ARM_SERVO_1 = 5
ARM_SERVO_2 = 6

# Default arm positions (placeholder values)
DEFAULT_POSITIONS = {
    ARM_SERVO_1: 6000,
    ARM_SERVO_2: 6000,
}

# Set default positions
for ch, val in DEFAULT_POSITIONS.items():
    maestro.setTarget(ch, val)

# Movement functions
def stop():
    maestro.setTarget(STRAIGHT, NEUTRAL)
    maestro.setTarget(ROTATE, NEUTRAL)

def move_forward():
    maestro.setTarget(STRAIGHT, FORWARD)

def move_backward():
    maestro.setTarget(STRAIGHT, BACKWARD)

def turn_left():
    maestro.setTarget(ROTATE, SPIN_LEFT)

def turn_right():
    maestro.setTarget(ROTATE, SPIN_RIGHT)

# Arm control placeholder functions
def arm_action_1():
    print("Arm action 1 triggered (U)")
    # maestro.setTarget(ARM_SERVO_1, VALUE)

def arm_action_2():
    print("Arm action 2 triggered (O)")
    # maestro.setTarget(ARM_SERVO_2, VALUE)

# Main control loop
def control_loop():
    print("Keyboard control started. Press ESC to quit.")
    try:
        while True:
            if keyboard.is_pressed("w"):
                move_forward()
            elif keyboard.is_pressed("s"):
                move_backward()
            elif keyboard.is_pressed("a"):
                turn_left()
            elif keyboard.is_pressed("d"):
                turn_right()
            elif keyboard.is_pressed("space"):
                stop()

            # Arm control keys
            elif keyboard.is_pressed("u"):
                arm_action_1()
            elif keyboard.is_pressed("o"):
                arm_action_2()

            elif keyboard.is_pressed("esc"):
                stop()
                break

            time.sleep(0.1)
    except KeyboardInterrupt:
        stop()
        print("Stopped by user.")

if __name__ == "__main__":
    control_loop()
