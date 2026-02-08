"""
Drowsiness Detection + UDP LED Alert using dlib
=================================================
This script uses your webcam to detect if you're getting drowsy (sleepy).
It watches your eyes - when your eyes stay closed for too long, it sends
"on" to the ESP8266 to turn on an alert LED.

How it works:
    1. dlib detects your face in the camera frame
    2. It finds 68 "landmark" points on your face (eyes, nose, mouth, etc.)
    3. We look at the 6 points around each eye
    4. We calculate the "Eye Aspect Ratio" (EAR) - a number that tells us
       how open or closed the eye is
    5. If EAR stays below a threshold for many frames --> you're drowsy!

What is EAR (Eye Aspect Ratio)?
    Imagine your eye from the front. It has a width and a height.

    EAR = (height of eye) / (width of eye)

    - Eyes wide open: EAR ~ 0.3 to 0.4 (tall opening)
    - Eyes closed:    EAR ~ 0.1 or less (flat line)

    The actual formula uses distances between specific eye landmarks:
        EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)

    Where p1-p6 are the 6 points around one eye:
        p1 (left corner) --- p2 (top-left) --- p3 (top-right) --- p4 (right corner)
                              p6 (bottom-left) -- p5 (bottom-right)

What is dlib?
    dlib is a C++ library (with Python bindings) for machine learning.
    It has a pre-trained model that can find 68 facial landmarks on a face.
    Points 36-41 = left eye, Points 42-47 = right eye.
"""

import os
import cv2
import dlib
import numpy as np
import socket
from scipy.spatial import distance as dist  # For calculating distances between points

# --- ESP8266 Configuration ---
DEFAULT_IP = "10.160.6.231"
ESP_PORT = 4210

ip_input = input(f"Enter ESP8266 IP address [{DEFAULT_IP}]: ").strip()
ESP_IP = ip_input if ip_input else DEFAULT_IP

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)

# --- Load dlib Models ---
# The shape predictor model file contains the trained data for finding
# 68 facial landmarks. It's a ~99MB file that must be downloaded separately.
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shape_predictor_68_face_landmarks.dat")

# Face detector: finds rectangular bounding boxes around faces in an image
detector = dlib.get_frontal_face_detector()

# Shape predictor: given a face rectangle, finds 68 specific points on the face
predictor = dlib.shape_predictor(MODEL_PATH)

# --- Eye Landmark Indices ---
# In dlib's 68-point model:
#   Left eye  = landmarks 36, 37, 38, 39, 40, 41
#   Right eye = landmarks 42, 43, 44, 45, 46, 47
LEFT_EYE = list(range(36, 42))
RIGHT_EYE = list(range(42, 48))

# --- Drowsiness Detection Thresholds ---
# If EAR drops below this value, we consider the eyes "closed"
# You may need to adjust this based on your face/camera
EAR_THRESHOLD = 0.25

# How many consecutive frames the eyes must be closed to trigger "drowsy"
# At 30 FPS, 20 frames = about 0.67 seconds of closed eyes
CONSEC_FRAMES = 20


def eye_aspect_ratio(eye_points):
    """
    Calculate the Eye Aspect Ratio (EAR) for one eye.

    The EAR formula:
        EAR = (|p2-p6| + |p3-p5|) / (2 * |p1-p4|)

    Where:
        p1, p4 = left and right corners of the eye (horizontal)
        p2, p6 = top-left and bottom-left of the eye (vertical)
        p3, p5 = top-right and bottom-right of the eye (vertical)

    Returns a float: ~0.3 when open, ~0.05 when closed
    """
    # Vertical distances (height of the eye opening)
    A = dist.euclidean(eye_points[1], eye_points[5])  # |p2 - p6|
    B = dist.euclidean(eye_points[2], eye_points[4])  # |p3 - p5|
    # Horizontal distance (width of the eye)
    C = dist.euclidean(eye_points[0], eye_points[3])  # |p1 - p4|
    # EAR formula
    return (A + B) / (2.0 * C)


# --- Open Camera ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit(1)

print("Camera opened. Drowsiness detection running. Press 'q' to quit.")

frame_counter = 0    # Counts how many consecutive frames eyes have been closed
prev_command = None   # Track last command to avoid sending duplicates

# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Mirror the frame for natural interaction
    frame = cv2.flip(frame, 1)

    # Convert to grayscale (dlib's face detector works on grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale image
    # Returns a list of rectangles, one for each detected face
    faces = detector(gray)

    for face in faces:
        # For each detected face, find the 68 facial landmarks
        landmarks = predictor(gray, face)

        # Extract the (x, y) coordinates for each eye
        left_eye = [(landmarks.part(i).x, landmarks.part(i).y) for i in LEFT_EYE]
        right_eye = [(landmarks.part(i).x, landmarks.part(i).y) for i in RIGHT_EYE]

        # Calculate EAR for each eye, then average them
        left_ear = eye_aspect_ratio(left_eye)
        right_ear = eye_aspect_ratio(right_eye)
        ear = (left_ear + right_ear) / 2.0

        # --- Draw eye outlines (green contours) ---
        # convexHull creates a smooth outline around the eye points
        left_hull = cv2.convexHull(np.array(left_eye))
        right_hull = cv2.convexHull(np.array(right_eye))
        cv2.drawContours(frame, [left_hull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [right_hull], -1, (0, 255, 0), 1)

        # Show the current EAR value on screen
        cv2.putText(frame, f"EAR: {ear:.2f}", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # --- Check for drowsiness ---
        if ear < EAR_THRESHOLD:
            # Eyes are closed this frame - increment counter
            frame_counter += 1
        else:
            # Eyes are open - reset counter
            frame_counter = 0

        if frame_counter >= CONSEC_FRAMES:
            # Eyes have been closed for too long --> DROWSY!
            cv2.putText(frame, "DROWSY!", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            command = "on"  # Turn on alert LED
        else:
            # Person is awake
            cv2.putText(frame, "Awake", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            command = "off"  # Turn off alert LED

        # --- Send UDP command only when state changes ---
        if command != prev_command:
            sock.sendto(command.encode(), (ESP_IP, ESP_PORT))
            try:
                data, _ = sock.recvfrom(1024)
                print(f"ESP: {data.decode()}")
            except socket.timeout:
                pass
            prev_command = command

    # Display the frame
    cv2.imshow("Drowsiness Detection", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
sock.close()
