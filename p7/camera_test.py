"""
Simple Camera Test using OpenCV
================================
This script opens your webcam and shows a live video feed in a window.
It's the simplest way to test if your camera is working with OpenCV.

What is OpenCV?
    OpenCV (Open Computer Vision) is a library that lets you work with
    images and video in Python. You can capture from cameras, process
    images, detect faces, and much more.

How a camera feed works:
    A video is just a series of pictures (called "frames") shown very fast.
    Typically 30 frames per second (30 FPS). This script reads one frame
    at a time in a loop and displays it on screen.
"""

# cv2 is the OpenCV library for Python
import cv2

# --- Open the Camera ---
# VideoCapture(0) opens the default camera (0 = first camera)
# If you have multiple cameras, try 1, 2, etc.
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera")
    exit(1)

print("Camera opened. Press 'q' to quit.")

# --- Main Loop: Read and display frames ---
while True:
    # cap.read() captures one frame from the camera
    # ret = True if the frame was captured successfully, False otherwise
    # frame = the actual image (a NumPy array of pixel values)
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    # Display the frame in a window titled "Camera"
    # This creates a GUI window that shows the image
    cv2.imshow("Camera", frame)

    # Wait 1 millisecond for a key press
    # If the key pressed is 'q', break out of the loop
    # The "& 0xFF" part is needed on some systems to correctly read the key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
# Release the camera so other programs can use it
cap.release()
# Close all OpenCV windows
cv2.destroyAllWindows()
