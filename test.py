import cv2
import numpy as np
import time
import pyrealsense2 as rs
from FinalProject.maestro import Controller

# Maestro channel assignments
LEFT_WHEEL = 0
RIGHT_WHEEL = 1

# Servo constants
NEUTRAL = 6000
FORWARD = 5000
BACKWARD = 7000

# Tracker for markers
passed_marker_ids = []

# Initialize Maestro controller
maestro = Controller()
maestro.setTarget(LEFT_WHEEL, NEUTRAL)
maestro.setTarget(RIGHT_WHEEL, NEUTRAL)

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
time.sleep(1)

# ArUco marker setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
camera_matrix = np.array([[615, 0, 320], [0, 615, 240], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

# Movement helpers
def stop():
    maestro.setTarget(LEFT_WHEEL, NEUTRAL)
    maestro.setTarget(RIGHT_WHEEL, NEUTRAL)

def turn_left(duration=0.5):
    print("Turning left...")
    maestro.setTarget(LEFT_WHEEL, BACKWARD)
    maestro.setTarget(RIGHT_WHEEL, FORWARD)
    time.sleep(duration)
    stop()

def turn_right(duration=0.5):
    print("Turning right...")
    maestro.setTarget(LEFT_WHEEL, FORWARD)
    maestro.setTarget(RIGHT_WHEEL, BACKWARD)
    time.sleep(duration)
    stop()

def move_forward(duration=1.0):
    print("Moving forward...")
    maestro.setTarget(LEFT_WHEEL, FORWARD)
    maestro.setTarget(RIGHT_WHEEL, FORWARD)
    time.sleep(duration)
    stop()

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
                    turn_left(0.4)
                else:
                    turn_right(0.4)

                move_forward(1.2)
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
