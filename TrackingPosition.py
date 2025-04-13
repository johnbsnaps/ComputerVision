import cv2
import numpy as np
import time
import pyrealsense2 as rs
from maestro import Controller
import threading

# Maestro channel assignments
STRAIGHT = 0
ROTATE = 1

global MOVING
MOVING = False


# Servo constants
PAN = 3
TILT = 4
PAN_CENTER = 6000
TILT_CENTER = 4400
PAN_RANGE = 300  # how far to pan left/right
TILT_RANGE = 200  # how far to tilt up/down

# Current position tracking
current_pan = PAN_CENTER
current_tilt = TILT_CENTER

NEUTRAL = 6000
FORWARD = 5200  
BACKWARD = 7000
SPIN_SPEED_RIGHT = 5200
SPIN_SPEED_LEFT = 6850

# Tracker for markers
passed_marker_ids = []
robot_positions = []

# Initialize Maestro controller
maestro = Controller()
maestro.setTarget(STRAIGHT, NEUTRAL)
maestro.setTarget(ROTATE, NEUTRAL)
maestro.setTarget(PAN, PAN_CENTER)
maestro.setTarget(TILT, TILT_CENTER)

# Initializes RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
time.sleep(1)

## Positional Data Functions
def get_camera_position_from_marker(marker_world_pos, rvec, tvec):
    # Convert rotation vector to rotation matrix
    R_ct, _ = cv2.Rodrigues(rvec)
    tvec = tvec.reshape((3, 1))

    # Invert the transformation
    R_tc = R_ct.T
    t_tc = -np.dot(R_tc, tvec)

    # Convert camera position into world space
    marker_x, marker_y = marker_world_pos
    marker_world = np.array([[marker_x], [marker_y], [0.0]])

    camera_world = marker_world + t_tc[0:3]
    return float(camera_world[0]), float(camera_world[1])


# Define known marker positions (example: marker_id: (x, y))
marker_positions = {
    0: (0.0, 0.0),
    1: (1.0, 0.0),
    2: (0.0, 1.0),
    # Add more as needed
}

# ArUco marker
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

camera_matrix = np.array([[615, 0, 320], [0, 615, 240], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

# Movement helpers (executed in a separate thread)
def stop():
    maestro.setTarget(STRAIGHT, NEUTRAL)
    maestro.setTarget(ROTATE, NEUTRAL)
    time.sleep(2)

def turn_left(duration=0.5):
    print("Turning left...")
    maestro.setTarget(ROTATE, SPIN_SPEED_LEFT)
    time.sleep(duration+.02)
    stop()

def turn_right(duration=0.5):
    print("Turning right...")
    maestro.setTarget(ROTATE, SPIN_SPEED_RIGHT)
    time.sleep(duration - 0.18)
    stop()

def move_forward(duration=1.0):
    print("Moving forward...")
    maestro.setTarget(STRAIGHT, FORWARD)
    time.sleep(duration)
    stop()

def center_marker_in_frame(frame, corners, threshold=10):
    global current_pan, current_tilt

    h, w, _ = frame.shape
    center_x = w // 2
    center_y = h // 2

    marker = corners[0][0]
    marker_x = int(np.mean(marker[:, 0]))
    marker_y = int(np.mean(marker[:, 1]))
    
    offset_x = marker_x - center_x
    offset_y = marker_y - center_y
    
    print(offset_x)
    print(offset_y)

    # Update pan/tilt only if offset is above threshold
    moved = False
    if abs(offset_x) > threshold:
        current_pan -= int(offset_x * 0.4)
        current_pan = max(min(current_pan, PAN_CENTER + PAN_RANGE), PAN_CENTER - PAN_RANGE)
        maestro.setTarget(PAN, current_pan)
        moved = True

    """ if abs(offset_y) > threshold:
        current_tilt += int(offset_y * 0.4)
        current_tilt = max(min(current_tilt, TILT_CENTER + TILT_RANGE), TILT_CENTER - TILT_RANGE)
        maestro.setTarget(TILT, current_tilt)
        moved = True """

    return not moved  # Return True when centered

#values for movement

global FACING

global ROTATE_90
global FORWARD_4_FEET
global FORWARD_1_FOOT
global ROTATE_45
global FORWARD_2_FEET
global CAMERA_LEFT
global CAMERA_RIGHT

FACING = "FORWARD"
FIRST_MOVE = 2.4
ROTATE_90_LEFT = 2.3
ROTATE_90_RIGHT = 2.2
FORWARD_4_FEET = 1.7
FORWARD_1_FOOT = 1.5
FORWARD_2_FEET = 2.6
ROTATE_45 = 1.5

CAMERA_LEFT = 7600
CAMERA_RIGHT = 4400


def pass_on_left():
    global MOVING
    MOVING = True  # Start movement
    # Create a movement sequence for the left side
    turn_left(ROTATE_90_LEFT)
    move_forward(FORWARD_1_FOOT)
    turn_right(ROTATE_90_RIGHT)
    move_forward(FORWARD_4_FEET)
    turn_right(ROTATE_90_RIGHT)
    move_forward(FORWARD_1_FOOT)
    turn_left(ROTATE_90_LEFT)
    MOVING = False  # Movement completed
    print(f"Done Moving, MOVING is now {MOVING}!" )


def pass_on_right():
    global MOVING
    MOVING = True  # Start movement
    # Create a movement sequence for the right side
    turn_right(ROTATE_90)
    move_forward(FORWARD_1_FOOT)
    turn_left(ROTATE_90)
    move_forward(FORWARD_4_FEET)
    turn_left(ROTATE_90)
    move_forward(FORWARD_1_FOOT)
    turn_right(ROTATE_90)
    MOVING = False  # Movement completed
    print(f"Done Moving, MOVING is now {MOVING}!" )
    
def zig_left():
    global FACING
    if (FACING == "FORWARD"):
        turn_left(ROTATE_45)
        move_forward(FIRST_MOVE)
        turn_right(ROTATE_90_RIGHT)
        move_forward(FORWARD_2_FEET)
    elif (FACING == "LEFT"):
        move_forward(FORWARD_2_FEET)
        turn_right(ROTATE_90_RIGHT)
        move_forward(FORWARD_2_FEET)
    elif (FACING == "RIGHT"):
        turn_left(ROTATE_90_LEFT)
        move_forward(FORWARD_2_FEET)
        turn_right(ROTATE_90_RIGHT)
        move_forward(FORWARD_2_FEET)
    maestro.setTarget(PAN, CAMERA_LEFT)
    FACING = "RIGHT"
        

def zig_right():
    global FACING
    if (FACING == "FORWARD"):
        turn_right(ROTATE_45)
        move_forward(FIRST_MOVE)
        turn_left(ROTATE_90_LEFT)
        move_forward(FORWARD_2_FEET)
    elif (FACING == "LEFT"):
        turn_right(ROTATE_90_RIGHT)
        move_forward(FORWARD_2_FEET)
        turn_left(ROTATE_90_LEFT)
        move_forward(FORWARD_2_FEET)
    elif (FACING == "RIGHT"):
        move_forward(FORWARD_2_FEET)
        turn_left(ROTATE_90_LEFT)
        move_forward(FORWARD_2_FEET)
    maestro.setTarget(PAN, CAMERA_RIGHT)
    FACING = "LEFT"


# Main loop
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Always show the camera feed
        cv2.imshow("ArUco Navigation", frame)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None and MOVING == False:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            print("Detected marker IDs:", ids)
            
            idx = 0
            centered = False
            centering_iterations = 25
            
            for x in range (centering_iterations):
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                frame = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                new_corners, new_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                if new_corners and new_ids is not None and len(new_ids) > idx:
                    center_marker_in_frame(frame, [new_corners[idx]])
                    time.sleep(.1)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            for marker_id in ids:
                if marker_id in passed_marker_ids:
                    continue  # Skip if already passed

                # Estimate pose (marker size: 0.055m)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[0]], 0.055, camera_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                x, y, _ = tvec[0][0]
                
                # Draw the position of the marker on the camera window
                cv2.putText(frame, f"Marker {marker_id} X: {x:.2f} Y: {y:.2f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Optionally, display the position near the marker
                for corner in corners:
                    for point in corner[0]:
                        cv2.circle(frame, tuple(np.int32(point)), 5, (0, 0, 255), -1)  # Mark the corner
                    marker_x, marker_y = np.mean(corner[0], axis=0)  # Get center of the marker
                    cv2.putText(frame, f"X: {x:.2f} Y: {y:.2f}", (int(marker_x), int(marker_y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)  # Display on marker center


                # Determine left/right side to pass
                side = "left" if marker_id % 2 else "right"
                cv2.putText(frame, f"Pass on the {side}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                print(f"Passing marker {marker_id} on the {side}, Pos: X={x:.2f}, Y={y:.2f}")

                # Call movement functions in a separate thread
                if side == "left":
                    zig_left()
                    #threading.Thread(target=zig_left).start()
                else:
                    #threading.Thread(target=zig_right).start()
                    zig_right()

                passed_marker_ids.append(marker_id)

                # Print current position after passing a marker
                print(f"Current Position (after passing Marker {marker_id}): X={x:.2f}, Y={y:.2f}")

                # Finish if all markers are passed
                if len(passed_marker_ids) >= 4:
                    print("\nCourse complete. Final Robot Positions:")
                    for mid, rx, ry in robot_positions:
                        print(f"  Near Marker {mid}: X = {rx:.2f}, Y = {ry:.2f}")
                    cv2.putText(frame, "FINISHED", (10, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    time.sleep(2)
                    raise StopIteration

                break  # Only handle one marker per frame

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except StopIteration:
    pass
finally:
    print("Shutting down.")
    pipeline.stop()
    cv2.destroyAllWindows()
    stop()
    maestro.close()
