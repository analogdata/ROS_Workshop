"""
Finger Counting + UDP LED Control using MediaPipe
===================================================
This script uses your webcam and Google's MediaPipe AI model to detect
your hand and count how many fingers are raised.

- Show 2 fingers --> sends "on"  to ESP8266 --> LED turns ON
- Show 3 fingers --> sends "off" to ESP8266 --> LED turns OFF

What is MediaPipe?
    MediaPipe is a library by Google that provides pre-trained AI models
    for detecting hands, faces, poses, etc. in images/video. We use the
    "Hand Landmarker" model which detects 21 key points on your hand.

How finger counting works:
    Each finger has a "tip" landmark and a "joint" landmark.
    - If the tip is ABOVE the joint (on screen), the finger is OPEN
    - If the tip is BELOW the joint, the finger is CLOSED (folded)
    The thumb is special - it moves sideways, so we compare X instead of Y.

    Hand Landmark Map (21 points):
        4  = Thumb tip         3  = Thumb IP joint
        8  = Index tip         6  = Index PIP joint
        12 = Middle tip        10 = Middle PIP joint
        16 = Ring tip          14 = Ring PIP joint
        20 = Pinky tip         18 = Pinky PIP joint
"""

import os
import cv2
import mediapipe as mp
import socket

# --- ESP8266 Configuration ---
DEFAULT_IP = "10.160.6.231"  # Default IP of the ESP8266
ESP_PORT = 4210               # Must match the port in the Arduino code

# Ask user for ESP IP, or use default
ip_input = input(f"Enter ESP8266 IP address [{DEFAULT_IP}]: ").strip()
ESP_IP = ip_input if ip_input else DEFAULT_IP

# Create a UDP socket for sending commands to the ESP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)  # Short timeout since we check every frame

# --- MediaPipe Hand Landmarker Setup (New Tasks API) ---
# MediaPipe 0.10+ uses a "Tasks API" instead of the old mp.solutions API
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Path to the pre-trained hand landmark model file
# This .task file must be downloaded separately (see docs)
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")

# Configure the hand landmarker
options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=VisionRunningMode.VIDEO,  # VIDEO mode for processing video frames
    num_hands=1,                           # Detect only 1 hand
    min_hand_detection_confidence=0.7,     # 70% confidence needed to detect a hand
    min_tracking_confidence=0.7,           # 70% confidence to keep tracking
)

# --- Finger Tip Landmark Indices ---
# These are the landmark IDs for each fingertip in MediaPipe's 21-point hand model
# Thumb=4, Index=8, Middle=12, Ring=16, Pinky=20
TIP_IDS = [4, 8, 12, 16, 20]

# --- Hand Skeleton Connections (for drawing lines between landmarks) ---
# Each tuple (a, b) means "draw a line from landmark a to landmark b"
HAND_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),       # Thumb
    (0,5),(5,6),(6,7),(7,8),       # Index finger
    (0,9),(9,10),(10,11),(11,12),  # Middle finger
    (0,13),(13,14),(14,15),(15,16),# Ring finger
    (0,17),(17,18),(18,19),(19,20),# Pinky
    (5,9),(9,13),(13,17),          # Palm connections
]

# --- Open Camera ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit(1)

print("Camera opened. Show 2 fingers = LED ON, 3 fingers = LED OFF. Press 'q' to quit.")

prev_command = None  # Track the last command sent (to avoid sending duplicates)
frame_ts = 0         # Fake timestamp for MediaPipe (it needs increasing timestamps)

# Create the hand landmarker and start processing
with HandLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Flip the frame horizontally (mirror effect, so it feels natural)
        frame = cv2.flip(frame, 1)

        # Convert BGR (OpenCV format) to RGB (MediaPipe format)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Wrap the frame as a MediaPipe Image object
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # Increment timestamp (~33ms per frame = ~30 FPS)
        frame_ts += 33

        # Run hand detection on this frame
        result = landmarker.detect_for_video(mp_image, frame_ts)

        finger_count = 0
        h, w, _ = frame.shape  # Get frame dimensions for converting normalized coords

        # Check if any hands were detected
        if result.hand_landmarks:
            for hand_lms in result.hand_landmarks:
                lm = hand_lms  # List of 21 landmarks

                # --- Draw landmarks (green dots) ---
                for idx, point in enumerate(lm):
                    # Landmarks are normalized (0.0 to 1.0), multiply by width/height
                    cx, cy = int(point.x * w), int(point.y * h)
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

                # --- Draw skeleton lines (white) ---
                for c1, c2 in HAND_CONNECTIONS:
                    x1, y1 = int(lm[c1].x * w), int(lm[c1].y * h)
                    x2, y2 = int(lm[c2].x * w), int(lm[c2].y * h)
                    cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)

                # --- Count Fingers ---
                # Thumb: moves sideways, so compare X position of tip vs IP joint
                if lm[TIP_IDS[0]].x < lm[TIP_IDS[0] - 1].x:
                    finger_count += 1

                # Index, Middle, Ring, Pinky: compare Y position
                # If tip (e.g., landmark 8) is ABOVE the PIP joint (e.g., landmark 6),
                # the finger is open. Note: in image coordinates, UP = smaller Y value.
                for i in range(1, 5):
                    if lm[TIP_IDS[i]].y < lm[TIP_IDS[i] - 2].y:
                        finger_count += 1

        # --- Display finger count on screen ---
        cv2.putText(frame, f"Fingers: {finger_count}", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        # --- Decide what command to send ---
        command = None
        if finger_count == 2:
            command = "on"
            cv2.putText(frame, "LED ON", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        elif finger_count == 3:
            command = "off"
            cv2.putText(frame, "LED OFF", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # --- Send UDP command only when it CHANGES ---
        # This prevents flooding the ESP with the same command every frame
        if command and command != prev_command:
            sock.sendto(command.encode(), (ESP_IP, ESP_PORT))
            try:
                data, _ = sock.recvfrom(1024)
                print(f"ESP: {data.decode()}")
            except socket.timeout:
                pass
            prev_command = command

        # Show the frame with all the drawings
        cv2.imshow("Finger Control", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
sock.close()
