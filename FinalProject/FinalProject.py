import cv2
import numpy as np
import time
import pickle
import pyrealsense2 as rs
from maestro import Controller
import threading



# Initialize RealSense pipeline to setup camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

time.sleep(1)

# Maestro channel assignments
STRAIGHT = 0
ROTATE = 1
PAN = 3
TILT = 4
RIGHT_ARM = 5

# Maestro Server Speeds
NEUTRAL = 6000
FORWARD = 5200  
BACKWARD = 7000
SPIN_SPEED_RIGHT = 5200
SPIN_SPEED_LEFT = 6850
PAN_CENTER = 6000
TILT_CENTER = 6000
ARM_DOWN = 4500
ARM_UP = 7500

# Initialize Maestro controller with starting values
maestro = Controller()
maestro.setTarget(STRAIGHT, NEUTRAL)
maestro.setTarget(ROTATE, NEUTRAL)
maestro.setTarget(PAN, PAN_CENTER)
maestro.setTarget(TILT, TILT_CENTER)
maestro.setTarget(RIGHT_ARM, ARM_DOWN)

# ArUco marker setup, loads the library, sets default viewing info, assigns 3D space values and then tells
# the camera there is no distortion

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

camera_matrix = np.array([[615, 0, 320], [0, 615, 240], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

# Boolean Values to track current mission progress
IDENTIFIED_OBJECT = False
FOUND_MARKER = False
ARRIVED_AT_MARKER = False
DROPPED_RING = False
FOUND_START = False
AT_START = False
RIGHT_ARM_UP = False


# Globals for Identifying Object
TARGET_ID = None
trained_objects = {}
orb = None
bf = None


# Sets up the identifier to be used
def identifier_setup():
    global trained_objects, orb, bf

    # Load trained objects
    with open("trainedObjects.pkl", "rb") as f:
        raw_objects = pickle.load(f)

    trained_objects = {}
    for obj_id, obj_data in raw_objects.items():
        # Rebuild cv2.KeyPoint objects from serialized format
        kp_tuples = obj_data["keypoints"]
        keypoints = [
            cv2.KeyPoint(pt[0], pt[1], size, angle, response, octave, class_id)
            for (pt, size, angle, response, octave, class_id) in kp_tuples
        ]

        trained_objects[obj_id] = {
            "name": obj_data["name"],
            "keypoints": keypoints,
            "descriptors": obj_data["descriptors"]
        }

    # Initialize ORB and BFMatcher
    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    print("Identifier setup complete.")


# Functions for movement, remember to start in a different thread.

def stop():
    maestro.setTarget(STRAIGHT, NEUTRAL)
    maestro.setTarget(ROTATE, NEUTRAL)
    time.sleep(2)

def turn_left(duration=0.5):
    print("Turning left...")
    maestro.setTarget(ROTATE, SPIN_SPEED_LEFT)
    time.sleep(duration)
    stop()

def turn_right(duration=0.5):
    print("Turning right...")
    maestro.setTarget(ROTATE, SPIN_SPEED_RIGHT)
    time.sleep(duration)
    stop()

def move_forward(duration=1.0):
    print("Moving forward...")
    maestro.setTarget(STRAIGHT, FORWARD)
    time.sleep(duration)
    stop()

def move_arm():
    global RIGHT_ARM_UP
    if(RIGHT_ARM_UP):
        maestro.setTarget(RIGHT_ARM, ARM_DOWN)
        RIGHT_ARM_UP = False
    else: 
        maestro.setTarget(RIGHT_ARM, ARM_UP)
        RIGHT_ARM_UP = True

# Uses the setup Identifier function, and then looks for the object
def identify_object(frame):
    global TARGET_ID, IDENTIFIED_OBJECT
    global trained_objects, orb, bf  # Use setup globals

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp_frame, des_frame = orb.detectAndCompute(gray, None)

    if des_frame is None or len(des_frame) == 0:
        TARGET_ID = None
        IDENTIFIED_OBJECT = False
        return

    best_match_id = None
    best_match_count = 0

    for obj_id, obj_data in trained_objects.items():
        des_obj = obj_data["descriptors"]
        if des_obj is None:
            continue

        matches = bf.match(des_frame, des_obj)
        matches = sorted(matches, key=lambda x: x.distance)
        good_matches = [m for m in matches if m.distance < 60]

        if len(good_matches) > best_match_count:
            best_match_count = len(good_matches)
            best_match_id = obj_id

    if best_match_count > 15:
        TARGET_ID = best_match_id
        IDENTIFIED_OBJECT = True
        move_arm()
        print(f"Identified object ID: {TARGET_ID}")
    else:
        TARGET_ID = None
        IDENTIFIED_OBJECT = False
        
        
#Takes in the current frame, and the ID of the marker you are trying to find. Spins until it can find
#the marker and then centers the marker within the frame.
def find_markers(frame, target_id):
    global FOUND_MARKER

    # Detect markers in the frame
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        ids = ids.flatten()
        if target_id in ids:
            index = list(ids).index(target_id)
            corner = corners[index][0]

            # Calculate center of the marker
            center_x = int(np.mean(corner[:, 0]))
            frame_center_x = frame.shape[1] // 2

            # Define a margin of error for centering
            margin = 30

            if abs(center_x - frame_center_x) <= margin:
                print(f"Marker {target_id} centered.")
                FOUND_MARKER = True
                return True
            elif center_x < frame_center_x:
                # Marker is to the left, rotate left slowly
                print(f"Marker {target_id} found, adjusting left...")
                turn_left(duration=0.2)
            else:
                # Marker is to the right, rotate right slowly
                print(f"Marker {target_id} found, adjusting right...")
                turn_right(duration=0.2)
            return False
        else:
            # Marker not in visible IDs, keep rotating
            print(f"Marker {target_id} not yet found, rotating...")
            turn_left(duration=0.2)
            return False
    else:
        # No markers detected at all
        print("No markers detected, rotating...")
        turn_left(duration=0.2)
        return False

## Detects the markers we are supposed to be moving towards, then if we are too far away, checks to make sure
## We are still in the center, if we are, then moves forward a bit, otherwise moves side to side to
## Keep centered.
def move_toward_marker(frame, marker_id):
    global ARRIVED_AT_MARKER
    global AT_START

    # Detect markers again
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None and marker_id in ids.flatten():
        index = list(ids.flatten()).index(marker_id)
        corner = corners[index][0]

        # Calculate center and area of the marker
        center_x = int(np.mean(corner[:, 0]))
        frame_center_x = frame.shape[1] // 2
        margin = 20

        # Compute marker area
        marker_area = cv2.contourArea(corner.astype(np.int32))
        frame_area = frame.shape[0] * frame.shape[1]
        marker_percentage = (marker_area / frame_area) * 100

        print(f"Marker size: {marker_percentage:.2f}% of frame.")

        # Check if the marker is big enough (i.e., we're close enough)
        if marker_percentage >= 15:
            if (marker_id == 0):
                print("Back at the Start!")
                AT_START == True
                return True
            else:
                print("Arrived at marker.")
                ARRIVED_AT_MARKER = True
                stop()
                return True

        # Adjust position slightly if off-center
        if abs(center_x - frame_center_x) <= margin:
            print("Moving forward toward marker...")
            move_forward(duration=0.3)
        elif center_x < frame_center_x:
            print("Marker drifting left... adjusting.")
            turn_left(duration=0.1)
        else:
            print("Marker drifting right... adjusting.")
            turn_right(duration=0.1)

        return False

    else:
        print("Lost sight of marker while moving forward!")
        stop()
        return False



def drop_ring():
    global DROPPED_RING
    DROPPED_RING = True
    move_arm()
    print("Dropping Ring")
    time.sleep(2)




def main():
    identifier_setup()
    try:
        while True:
            # Wait for a new frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # Convert RealSense frame to NumPy array
            frame = np.asanyarray(color_frame.get_data())

            # Show the frame (optional)
            cv2.imshow("RealSense Camera Feed", frame)
            
            if (IDENTIFIED_OBJECT):
                if (FOUND_MARKER):
                    if (ARRIVED_AT_MARKER):
                        if (DROPPED_RING):
                            if (FOUND_START):
                                if (AT_START):
                                    print("All Done!")
                                else: move_toward_marker(frame, 0)
                            else: find_markers(frame, 0)
                        else: drop_ring()
                    else: move_toward_marker(frame, TARGET_ID)
                else: find_markers(frame, TARGET_ID)
            else: identify_object(frame)

            # Exit loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        
main()
