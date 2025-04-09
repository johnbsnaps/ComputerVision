import cv2
import numpy as np
import time
import pyrealsense2 as rs
from maestro import Controller

# Maestro channel assignments
STRAIGHT = 0
ROTATE = 1

# Servo constants
PAN = 3
TILT = 4
PAN_CENTER = 6000
TILT_CENTER = 6000
PAN_RANGE = 300  # how far to pan left/right
TILT_RANGE = 200  # how far to tilt up/down

# Current position tracking
current_pan = PAN_CENTER
current_tilt = TILT_CENTER

NEUTRAL = 6000
FORWARD = 5000  
BACKWARD = 7000

# Tracker for markers
passed_marker_ids = []

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

# ArUco marker
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

camera_matrix = np.array([[615, 0, 320], [0, 615, 240], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

# Movement helpers
def stop():
    maestro.setTarget(STRAIGHT, NEUTRAL)
    maestro.setTarget(ROTATE, NEUTRAL)

def turn_left(duration=0.5):
    print("Turning left...")
    maestro.setTarget(ROTATE, BACKWARD)
    time.sleep(duration)
    stop()

def turn_right(duration=0.5):
    print("Turning right...")
    maestro.setTarget(ROTATE, FORWARD)
    time.sleep(duration)
    stop()

def move_forward(duration=1.0):
    print("Moving forward...")
    maestro.setTarget(STRAIGHT, FORWARD)
    time.sleep(duration)
    stop()
     
def center_marker_in_frame(frame, corners, threshold=20):
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

    if abs(offset_y) > threshold:
        current_tilt += int(offset_y * 0.4)
        current_tilt = max(min(current_tilt, TILT_CENTER + TILT_RANGE), TILT_CENTER - TILT_RANGE)
        maestro.setTarget(TILT, current_tilt)
        moved = True

    return not moved  # Return True when centered


# Main loop
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            print("Detected marker IDs:", ids)
            
            idx = 0
            centered = False
            while not centered:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                frame = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                new_corners, new_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                if new_corners and new_ids is not None and len(new_ids) > idx:
                    centered = center_marker_in_frame(frame, [new_corners[idx]])

                cv2.imshow("Centering", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            

            for i, marker_id in enumerate(ids):
                if marker_id in passed_marker_ids:
                    continue  # Skip if already passed

                # Estimate pose (marker size: 0.055m)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[i]], 0.055, camera_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                x, y, _ = tvec[0][0]
                cv2.putText(frame, f"Marker {marker_id} X: {x:.2f} Y: {y:.2f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Determine left/right side to pass
                side = "left" if marker_id % 2 else "right"
                cv2.putText(frame, f"Pass on the {side}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                print(f"Passing marker {marker_id} on the {side}, Pos: X={x:.2f}, Y={y:.2f}")

                # Turn then move forward
                if side == "left":
                    turn_left(1.4)
                    move_forward(1)
                    turn_right(1.4)
                else:
                    turn_right(1.4)
                    move_forward(1)
                    turn_left(1.4)
                move_forward(3)

                passed_marker_ids.append(marker_id)

                if len(passed_marker_ids) >= 4:
                    print("Finished course!")
                    cv2.putText(frame, "FINISHED", (10, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    time.sleep(2)
                    raise StopIteration

                break  # Only handle one marker per frame

        cv2.imshow("ArUco Navigation", frame)
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
