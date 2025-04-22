import cv2
import pickle
import numpy as np
import pyrealsense2 as rs

# Constants
NUM_OBJECTS_TO_TRAIN = 3
trained_objects = {}

# Setup ORB
orb = cv2.ORB_create(nfeatures=500)

# Initialize RealSense
try:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    use_realsense = True
except:
    print("Falling back to webcam.")
    cap = cv2.VideoCapture(0)
    use_realsense = False

# Mouse callback data
drawing = False
start_point = (0, 0)
end_point = (0, 0)
bbox_ready = False

def mouse_callback(event, x, y, flags, param):
    global drawing, start_point, end_point, bbox_ready
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_point = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            end_point = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        end_point = (x, y)
        bbox_ready = True

cv2.namedWindow("Draw Object")
cv2.setMouseCallback("Draw Object", mouse_callback)

object_count = 0

print("Please draw bounding boxes around 3 objects. Press ESC to cancel.")

while object_count < NUM_OBJECTS_TO_TRAIN:
    # Get frame
    if use_realsense:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
    else:
        ret, frame = cap.read()
        if not ret:
            continue

    display_frame = frame.copy()

    # Draw in-progress rectangle
    if drawing:
        cv2.rectangle(display_frame, start_point, end_point, (0, 255, 0), 2)

    cv2.putText(display_frame, f"Object {object_count+1}/3 - Draw box", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
    cv2.imshow("Draw Object", display_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC to quit
        break

    if bbox_ready:
        bbox_ready = False
        x1, y1 = start_point
        x2, y2 = end_point
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)

        roi = frame[y_min:y_max, x_min:x_max]
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = orb.detectAndCompute(gray_roi, None)

        if descriptors is not None:
            obj_name = input(f"Enter a name for object {object_count+1}: ")

            # Convert keypoints to a serializable format
            keypoints_serializable = [(
                kp.pt, kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            ) for kp in keypoints]

            trained_objects[object_count + 1] = {
                "name": obj_name,
                "keypoints": keypoints_serializable,
                "descriptors": descriptors
            }

            print(f"[{obj_name}] stored with {len(keypoints)} keypoints.")
            object_count += 1
        else:
            print("No features found, try again.")

# Save to file
if len(trained_objects) == NUM_OBJECTS_TO_TRAIN:
    with open("trainedObjects.pkl", "wb") as f:
        pickle.dump(trained_objects, f)
    print("Lazy teen memory activated. Object recognition enabled.")

# Cleanup
if use_realsense:
    pipeline.stop()
else:
    cap.release()

cv2.destroyAllWindows()
