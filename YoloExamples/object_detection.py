"""
YOLO Object Detection — Real-Time with Webcam
==============================================
This program uses Ultralytics YOLOv8 to detect objects in real-time
from your webcam feed. It draws bounding boxes around detected objects
and labels them with their class name and confidence score.

What is YOLO?
  YOLO = "You Only Look Once"
  It's an AI model that can detect multiple objects in an image in a
  single pass — making it extremely fast (real-time on most computers).

How it works:
  1. Capture a frame from the webcam
  2. Feed the frame to the YOLOv8 model
  3. The model returns a list of detected objects with:
     - Bounding box coordinates (x1, y1, x2, y2)
     - Class name (e.g., "person", "car", "dog")
     - Confidence score (0.0 to 1.0 — how sure the model is)
  4. Draw the boxes and labels on the frame
  5. Display the frame and repeat

Usage:
  uv run python YoloExamples/object_detection.py

Controls:
  q — Quit the program
  s — Save the current frame as a screenshot
"""

# ─── Imports ───────────────────────────────────────────────────────────────────

# cv2 (OpenCV) — captures webcam frames and draws boxes/text on images
import cv2

# ultralytics — provides the YOLO model for object detection
from ultralytics import YOLO

# datetime — used to generate timestamped filenames for screenshots
from datetime import datetime


# ─── Configuration ─────────────────────────────────────────────────────────────

# Which YOLO model to use:
#   yolov8n.pt — Nano    (fastest, least accurate, ~6 MB)
#   yolov8s.pt — Small   (good balance, ~22 MB)
#   yolov8m.pt — Medium  (slower, more accurate, ~50 MB)
#   yolov8l.pt — Large   (slow, very accurate, ~84 MB)
#   yolov8x.pt — XLarge  (slowest, most accurate, ~131 MB)
# The model file is downloaded automatically on first run.
MODEL_NAME = "yolov8n.pt"

# Minimum confidence threshold (0.0 to 1.0)
# Objects detected with confidence below this are ignored.
# Lower = more detections (but more false positives)
# Higher = fewer detections (but more reliable)
CONFIDENCE_THRESHOLD = 0.5

# Camera index (0 = default webcam, 1 = second camera, etc.)
CAMERA_INDEX = 0

# Display window name
WINDOW_NAME = "YOLOv8 Object Detection"


# ─── Color Palette ─────────────────────────────────────────────────────────────

# A list of colors (BGR format) for drawing boxes around different object classes.
# Each class gets a unique color so you can visually distinguish them.
COLORS = [
    (255, 0, 0),      # Blue
    (0, 255, 0),      # Green
    (0, 0, 255),      # Red
    (255, 255, 0),    # Cyan
    (255, 0, 255),    # Magenta
    (0, 255, 255),    # Yellow
    (128, 0, 255),    # Purple
    (255, 128, 0),    # Orange
    (0, 128, 255),    # Light Blue
    (128, 255, 0),    # Lime
]


def main():
    """Main function — loads model, opens camera, runs detection loop."""

    # ─── Step 1: Load the YOLO model ──────────────────────────────────────────
    # On first run, this downloads the model weights from the internet (~6 MB for nano).
    # After that, it loads from the local cache.
    print(f"Loading YOLO model: {MODEL_NAME}")
    print("(First run will download the model — this may take a moment)")
    model = YOLO(MODEL_NAME)
    print("Model loaded successfully!\n")

    # ─── Step 2: Open the webcam ──────────────────────────────────────────────
    # cv2.VideoCapture(0) opens the default camera.
    # The argument is the camera index (0 = first camera).
    print(f"Opening camera (index {CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("ERROR: Could not open camera!")
        print("  - Is a webcam connected?")
        print("  - Is another program using the camera?")
        print(f"  - Try changing CAMERA_INDEX (currently {CAMERA_INDEX})")
        return

    # Get the camera's resolution for display info
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened: {frame_width}x{frame_height}")
    print()
    print("─── Controls ───────────────────")
    print("  q — Quit")
    print("  s — Save screenshot")
    print("────────────────────────────────")
    print()

    # ─── Step 3: Detection loop ───────────────────────────────────────────────
    # This loop runs continuously until you press 'q'.
    frame_count = 0

    while True:
        # Read one frame from the camera
        # ret = True if frame was captured successfully, False otherwise
        # frame = the image as a NumPy array (height x width x 3 BGR channels)
        ret, frame = cap.read()

        if not ret:
            print("ERROR: Failed to read frame from camera.")
            break

        frame_count += 1

        # ─── Step 4: Run YOLO detection on the frame ─────────────────────────
        # model() runs the neural network on the image.
        # - conf: minimum confidence threshold
        # - verbose=False: don't print detection details to console every frame
        #
        # results is a list (one entry per image — we only have one image).
        results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)

        # Get the first (and only) result
        result = results[0]

        # ─── Step 5: Draw detections on the frame ────────────────────────────
        # result.boxes contains all detected objects.
        # Each box has:
        #   .xyxy  — bounding box coordinates [x1, y1, x2, y2]
        #   .conf  — confidence score (0.0 to 1.0)
        #   .cls   — class index (integer)
        #
        # result.names is a dictionary mapping class index → class name
        # e.g., {0: 'person', 1: 'bicycle', 2: 'car', ...}

        detection_count = 0

        for box in result.boxes:
            # Extract bounding box coordinates (convert to integers for drawing)
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Extract confidence score (how sure the model is)
            confidence = float(box.conf[0])

            # Extract class index and look up the class name
            class_id = int(box.cls[0])
            class_name = result.names[class_id]

            # Pick a color for this class (cycle through the palette)
            color = COLORS[class_id % len(COLORS)]

            # Draw the bounding box rectangle
            # Arguments: image, top-left corner, bottom-right corner, color, thickness
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Create the label text: "person 0.95"
            label = f"{class_name} {confidence:.2f}"

            # Calculate text size so we can draw a filled background behind it
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )

            # Draw a filled rectangle behind the text (for readability)
            cv2.rectangle(
                frame,
                (x1, y1 - text_height - 10),
                (x1 + text_width, y1),
                color,
                -1  # -1 = filled rectangle
            )

            # Draw the label text (white on colored background)
            cv2.putText(
                frame,
                label,
                (x1, y1 - 5),                  # Position (slightly above the box)
                cv2.FONT_HERSHEY_SIMPLEX,       # Font
                0.6,                            # Font scale
                (255, 255, 255),                # White text
                2                               # Thickness
            )

            detection_count += 1

        # ─── Step 6: Draw info overlay ────────────────────────────────────────
        # Show the number of detected objects on the frame
        info_text = f"Objects: {detection_count} | Frame: {frame_count} | Press 'q' to quit"
        cv2.putText(
            frame,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),   # Green text
            2
        )

        # ─── Step 7: Display the frame ───────────────────────────────────────
        cv2.imshow(WINDOW_NAME, frame)

        # ─── Step 8: Handle keyboard input ───────────────────────────────────
        # cv2.waitKey(1) waits 1 millisecond for a key press.
        # & 0xFF masks the result to get the ASCII value.
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            # 'q' pressed — quit the program
            print("\nQuitting...")
            break

        elif key == ord('s'):
            # 's' pressed — save the current frame as a screenshot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"YoloExamples/screenshot_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Screenshot saved: {filename}")

    # ─── Cleanup ──────────────────────────────────────────────────────────────
    # Release the camera and close all OpenCV windows.
    # Always do this to free up the camera for other programs.
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released. Goodbye!")


# ─── Entry Point ──────────────────────────────────────────────────────────────
# This block runs only when you execute this file directly:
#   uv run python YoloExamples/object_detection.py
# It does NOT run if this file is imported by another script.
if __name__ == "__main__":
    main()
