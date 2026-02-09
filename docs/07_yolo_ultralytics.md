# YOLO & Ultralytics — Object Detection for Beginners

## Table of Contents

1. [What is Object Detection?](#1-what-is-object-detection)
2. [What is YOLO?](#2-what-is-yolo)
3. [The YOLO Family — History & Versions](#3-the-yolo-family--history--versions)
4. [What is Ultralytics?](#4-what-is-ultralytics)
5. [How YOLO Works Under the Hood](#5-how-yolo-works-under-the-hood)
6. [YOLOv8 Model Sizes — Which One to Pick?](#6-yolov8-model-sizes--which-one-to-pick)
7. [The 80 COCO Classes — What Can YOLO Detect?](#7-the-80-coco-classes--what-can-yolo-detect)
8. [Understanding the Output](#8-understanding-the-output)
9. [Our object_detection.py — Code Walkthrough](#9-our-object_detectionpy--code-walkthrough)
10. [Beyond Detection — What Else Can YOLO Do?](#10-beyond-detection--what-else-can-yolo-do)
11. [Real-World Project Ideas — Image Processing](#11-real-world-project-ideas--image-processing)
12. [Real-World Project Ideas — Embedded & IoT](#12-real-world-project-ideas--embedded--iot)
13. [YOLO + ROS2 — Connecting Vision to Robots](#13-yolo--ros2--connecting-vision-to-robots)
14. [YOLO + ESP8266 — Vision-Triggered Hardware](#14-yolo--esp8266--vision-triggered-hardware)
15. [Training Your Own YOLO Model](#15-training-your-own-yolo-model)
16. [Running YOLO on Edge Devices](#16-running-yolo-on-edge-devices)
17. [Common Terminology Glossary](#17-common-terminology-glossary)
18. [Commands Quick Reference](#18-commands-quick-reference)
19. [Troubleshooting](#19-troubleshooting)

---

## 1. What is Object Detection?

Object detection is a computer vision task where an AI model looks at an
image (or video frame) and answers two questions:

1. **What** objects are in the image? (classification)
2. **Where** are they? (localization)

```
┌─────────────────────────────────────────────┐
│                                             │
│    ┌──────────┐                             │
│    │  person  │     ┌─────────┐             │
│    │  0.97    │     │  dog    │             │
│    │          │     │  0.89   │             │
│    │          │     └─────────┘             │
│    └──────────┘                             │
│                        ┌──────────┐         │
│                        │  car     │         │
│                        │  0.94    │         │
│                        └──────────┘         │
│                                             │
└─────────────────────────────────────────────┘
```

Each detection gives you:
- **Bounding box** — A rectangle around the object (x1, y1, x2, y2)
- **Class name** — What the object is ("person", "dog", "car")
- **Confidence score** — How sure the model is (0.0 to 1.0)

### Object Detection vs. Other Vision Tasks

| Task | What it does | Output |
|------|-------------|--------|
| **Image Classification** | "What is in this image?" | One label: "cat" |
| **Object Detection** | "What objects and where?" | Multiple boxes + labels |
| **Segmentation** | "Exact pixel boundaries?" | Pixel-level masks |
| **Pose Estimation** | "Where are body joints?" | Skeleton keypoints |

YOLO can do **all four** of these tasks!

---

## 2. What is YOLO?

**YOLO = You Only Look Once**

Before YOLO (2015), object detection was slow. Previous methods worked in
two stages:

```
Old Method (R-CNN, 2014):
  Step 1: Scan the image and propose ~2000 "regions of interest"
  Step 2: Run a classifier on EACH region separately
  Speed: ~0.05 FPS (20 seconds per image!)

YOLO (2015):
  Step 1: Look at the ENTIRE image ONCE with a single neural network
  Output: All objects, boxes, and confidence scores in one pass
  Speed: ~45 FPS (real-time!)
```

### Why "You Only Look Once"?

The name comes from the key insight: instead of looking at thousands of
small regions one by one, YOLO processes the **entire image in a single
forward pass** through the neural network. One look = all detections.

### The Grid System

YOLO divides the image into a grid (e.g., 13×13 cells). Each cell is
responsible for detecting objects whose center falls within that cell:

```
┌───┬───┬───┬───┬───┐
│   │   │   │   │   │
├───┼───┼───┼───┼───┤
│   │   │ ● │   │   │  ● = Object center falls in this cell
├───┼───┼───┼───┼───┤     → This cell predicts the bounding box
│   │   │   │   │   │
├───┼───┼───┼───┼───┤
│   │   │   │   │   │
├───┼───┼───┼───┼───┤
│   │   │   │   │   │
└───┴───┴───┴───┴───┘

Each cell predicts:
  - B bounding boxes (x, y, width, height)
  - Confidence score for each box
  - Class probabilities (person? car? dog?)
```

---

## 3. The YOLO Family — History & Versions

| Version | Year | Creator | Key Innovation |
|---------|------|---------|---------------|
| **YOLOv1** | 2015 | Joseph Redmon | First real-time detector — single-pass design |
| **YOLOv2** | 2016 | Joseph Redmon | Batch normalization, anchor boxes, higher resolution |
| **YOLOv3** | 2018 | Joseph Redmon | Multi-scale detection (detects small + large objects) |
| **YOLOv4** | 2020 | Alexey Bochkovskiy | Bag of tricks: mosaic augmentation, CSP backbone |
| **YOLOv5** | 2020 | Ultralytics | PyTorch-based, easy to use, great community |
| **YOLOv6** | 2022 | Meituan | Optimized for industrial deployment |
| **YOLOv7** | 2022 | WongKinYiu | Extended efficient layer aggregation |
| **YOLOv8** | 2023 | Ultralytics | Anchor-free, unified API for detect/segment/pose/classify |
| **YOLOv9** | 2024 | WongKinYiu | Programmable gradient information (PGI) |
| **YOLOv10** | 2024 | Tsinghua Univ. | NMS-free, end-to-end detection |
| **YOLO11** | 2024 | Ultralytics | Latest, improved accuracy and speed |

**We use YOLOv8** in this workshop because:
- It's the most mature and well-documented
- Ultralytics provides an incredibly easy Python API
- It supports detection, segmentation, pose, and classification
- Huge community and ecosystem

---

## 4. What is Ultralytics?

[Ultralytics](https://ultralytics.com) is the company behind YOLOv5, YOLOv8,
and YOLO11. They provide:

- **Pre-trained models** — Ready to use, no training needed
- **Python package** — `pip install ultralytics` (or `uv add ultralytics`)
- **CLI tool** — Run detection from the command line
- **Training pipeline** — Train on your own custom data
- **Export** — Convert models to ONNX, TensorRT, CoreML, etc.

### The Ultralytics Python API

The API is designed to be dead simple:

```python
from ultralytics import YOLO

# Load a model (downloads automatically on first use)
model = YOLO("yolov8n.pt")

# Detect objects in an image
results = model("photo.jpg")

# That's it! results contains all detections.
```

Three lines of code to detect objects. That's the power of Ultralytics.

### Downloading the YOLOv8n Model

The model auto-downloads on first use, but you can pre-download it:

```bash
# Option 1: From the YoloExamples directory (recommended)
cd ~/ROS_Workshop/YoloExamples
uv run python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
# Downloads yolov8n.pt (~6.2 MB) into the current directory

# Option 2: Using the Ultralytics CLI
uv run yolo detect predict model=yolov8n.pt source=0 show=True
# Downloads the model AND starts webcam detection immediately
```

After downloading, you should see:
```bash
ls -lh YoloExamples/yolov8n.pt
# -rw-r--r-- 1 user user 6.2M ... yolov8n.pt
```

> **Why pre-download?** If you're at a workshop with slow WiFi, downloading
> beforehand saves time. The `.pt` file is excluded from Git via `.gitignore`.

---

## 5. How YOLO Works Under the Hood

### The Neural Network Architecture

YOLOv8 is a **Convolutional Neural Network (CNN)** with three main parts:

```
Input Image (640×640×3)
        │
        ▼
┌─────────────────────┐
│     BACKBONE         │  ← Feature extraction
│  (CSPDarknet53)      │     Learns to recognize patterns:
│                      │     edges → textures → shapes → objects
│  Conv → Conv → Conv  │
│  Pool → Conv → Conv  │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│       NECK           │  ← Feature fusion
│  (FPN + PAN)         │     Combines features from different scales
│                      │     Small objects ← high-resolution features
│  Upsample + Concat   │     Large objects ← low-resolution features
│  Downsample + Concat  │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│       HEAD           │  ← Prediction
│  (Decoupled Head)    │     For each detected object:
│                      │     - Bounding box (x, y, w, h)
│  Classification head │     - Class probabilities
│  Regression head     │     - Objectness score
└─────────────────────┘
          │
          ▼
    Detected Objects
    [person 0.97, car 0.94, dog 0.89]
```

### What Happens Step by Step

1. **Input**: Your image is resized to 640×640 pixels
2. **Backbone**: Extracts features (edges, textures, shapes) at multiple scales
3. **Neck**: Combines multi-scale features (so it can detect tiny and huge objects)
4. **Head**: Predicts bounding boxes and class probabilities
5. **Post-processing**: Removes duplicate/overlapping detections (NMS)

### What is NMS (Non-Maximum Suppression)?

The model might detect the same object multiple times with slightly different
boxes. NMS removes the duplicates:

```
Before NMS:                    After NMS:
┌──────────┐                   ┌──────────┐
│┌────────┐│                   │          │
││ person ││  ← 3 overlapping  │  person  │  ← Keep only the best one
│└────────┘│     boxes for     │  0.97    │
└──────────┘     one person    └──────────┘
```

Algorithm:
1. Sort all boxes by confidence score (highest first)
2. Keep the highest-scoring box
3. Remove all other boxes that overlap with it by more than a threshold (IoU > 0.5)
4. Repeat for the next highest-scoring box

---

## 6. YOLOv8 Model Sizes — Which One to Pick?

Ultralytics provides 5 sizes of YOLOv8. They all detect the same 80 classes
but differ in speed vs. accuracy:

| Model | Size | Parameters | Speed (CPU) | Speed (GPU) | Accuracy (mAP) |
|-------|------|-----------|-------------|-------------|-----------------|
| **yolov8n** (Nano) | 6.2 MB | 3.2M | ~100ms | ~5ms | 37.3 |
| **yolov8s** (Small) | 22 MB | 11.2M | ~200ms | ~7ms | 44.9 |
| **yolov8m** (Medium) | 50 MB | 25.9M | ~400ms | ~10ms | 50.2 |
| **yolov8l** (Large) | 84 MB | 43.7M | ~700ms | ~13ms | 52.9 |
| **yolov8x** (XLarge) | 131 MB | 68.2M | ~1000ms | ~16ms | 53.9 |

### How to Choose

```
Need real-time on CPU (laptop/Raspberry Pi)?
  → yolov8n (Nano) — fastest, good enough for most demos

Need better accuracy, have a GPU?
  → yolov8s or yolov8m — good balance

Need maximum accuracy, speed doesn't matter?
  → yolov8l or yolov8x — research/production quality

Running on embedded (Jetson Nano, ESP32-CAM)?
  → yolov8n exported to TensorRT or ONNX
```

### What is mAP?

**mAP = mean Average Precision** — the standard metric for object detection.
- Higher is better (0 to 100)
- It measures how well the model finds objects AND how accurate the boxes are
- mAP 37.3 (nano) vs 53.9 (xlarge) means xlarge finds ~45% more objects correctly

For this workshop, **yolov8n (Nano)** is perfect — it runs in real-time
even on a laptop without a GPU.

---

## 7. The 80 COCO Classes — What Can YOLO Detect?

YOLOv8 is pre-trained on the **COCO dataset** (Common Objects in Context),
which contains 80 everyday object classes:

### People & Animals
`person`, `bicycle`, `car`, `motorcycle`, `airplane`, `bus`, `train`, `truck`,
`boat`, `bird`, `cat`, `dog`, `horse`, `sheep`, `cow`, `elephant`, `bear`,
`zebra`, `giraffe`

### Everyday Objects
`backpack`, `umbrella`, `handbag`, `tie`, `suitcase`, `frisbee`, `skis`,
`snowboard`, `sports ball`, `kite`, `baseball bat`, `baseball glove`,
`skateboard`, `surfboard`, `tennis racket`

### Food & Kitchen
`bottle`, `wine glass`, `cup`, `fork`, `knife`, `spoon`, `bowl`, `banana`,
`apple`, `sandwich`, `orange`, `broccoli`, `carrot`, `hot dog`, `pizza`,
`donut`, `cake`

### Furniture & Indoor
`chair`, `couch`, `potted plant`, `bed`, `dining table`, `toilet`, `tv`,
`laptop`, `mouse`, `remote`, `keyboard`, `cell phone`, `microwave`, `oven`,
`toaster`, `sink`, `refrigerator`, `book`, `clock`, `vase`, `scissors`,
`teddy bear`, `hair drier`, `toothbrush`

### Traffic
`traffic light`, `fire hydrant`, `stop sign`, `parking meter`, `bench`

> **Want to detect something not in this list?** You can train a custom
> YOLO model on your own dataset! See [Section 15](#15-training-your-own-yolo-model).

---

## 8. Understanding the Output

When you run YOLO, here's what you get back:

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
results = model("image.jpg")

# results is a list (one entry per image)
result = results[0]
```

### result.boxes — All Detected Objects

```python
for box in result.boxes:
    # Bounding box coordinates (top-left and bottom-right corners)
    x1, y1, x2, y2 = box.xyxy[0]
    # x1, y1 = top-left corner
    # x2, y2 = bottom-right corner

    # Confidence score (0.0 to 1.0)
    confidence = box.conf[0]    # e.g., 0.95

    # Class index (integer)
    class_id = box.cls[0]       # e.g., 0

    # Class name (look up from the names dictionary)
    class_name = result.names[int(class_id)]  # e.g., "person"
```

### Visual Explanation of Bounding Box Coordinates

```
(0,0) ──────────────────────────────── x
  │
  │     (x1, y1)
  │        ┌──────────────┐
  │        │              │
  │        │   "person"   │  height = y2 - y1
  │        │    0.95      │
  │        │              │
  │        └──────────────┘
  │                    (x2, y2)
  │
  y        width = x2 - x1
```

### Other Box Formats

```python
box.xyxy    # [x1, y1, x2, y2]  — corners (what we use)
box.xywh    # [cx, cy, w, h]    — center + width/height
box.xywhn   # [cx, cy, w, h]    — normalized (0-1) center + size
box.xyxyn   # [x1, y1, x2, y2]  — normalized (0-1) corners
```

---

## 9. Our object_detection.py — Code Walkthrough

Here's what our `YoloExamples/object_detection.py` does, step by step:

```
┌─────────────────────────────────────────────────────────────┐
│                    Program Flow                             │
│                                                             │
│  1. Load YOLO model (yolov8n.pt)                           │
│     └── Downloads ~6 MB on first run                       │
│                                                             │
│  2. Open webcam (cv2.VideoCapture)                          │
│     └── Check if camera is available                       │
│                                                             │
│  3. ┌── LOOP (runs every frame) ──────────────────────┐    │
│     │                                                  │    │
│     │  a. Read frame from camera                       │    │
│     │  b. Run YOLO detection on the frame              │    │
│     │  c. For each detected object:                    │    │
│     │     - Draw colored bounding box                  │    │
│     │     - Draw label with class name + confidence    │    │
│     │  d. Draw info overlay (object count, frame #)    │    │
│     │  e. Display the frame in a window                │    │
│     │  f. Check for key press:                         │    │
│     │     - 'q' → quit                                 │    │
│     │     - 's' → save screenshot                      │    │
│     │                                                  │    │
│     └──────────────────────────────────────────────────┘    │
│                                                             │
│  4. Release camera and close windows                        │
└─────────────────────────────────────────────────────────────┘
```

### Key Code Sections Explained

**Loading the model:**
```python
model = YOLO("yolov8n.pt")
```
This loads the pre-trained YOLOv8 Nano model. On first run, it downloads
the weights (~6.2 MB) from GitHub. After that, it loads from the local file.

**Running detection:**
```python
results = model(frame, conf=0.5, verbose=False)
```
- `frame` — The webcam image (NumPy array, 480×640×3)
- `conf=0.5` — Ignore detections below 50% confidence
- `verbose=False` — Don't print logs every frame

**Drawing boxes:**
```python
cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
```
OpenCV draws directly on the image array. The `2` is line thickness.

**The color palette:**
Each object class gets a different color so you can visually distinguish
a "person" box from a "car" box. We cycle through 10 colors using
`class_id % len(COLORS)`.

---

## 10. Beyond Detection — What Else Can YOLO Do?

YOLOv8 isn't just for detection. It supports **4 tasks** with the same API:

### Task 1: Object Detection (what we're doing)
```python
model = YOLO("yolov8n.pt")
results = model("image.jpg")
# → Bounding boxes + class names
```

### Task 2: Instance Segmentation
```python
model = YOLO("yolov8n-seg.pt")
results = model("image.jpg")
# → Pixel-perfect masks for each object
```
Instead of a rectangle, you get the **exact shape** of each object.
Useful for: background removal, autonomous driving, medical imaging.

### Task 3: Pose Estimation
```python
model = YOLO("yolov8n-pose.pt")
results = model("image.jpg")
# → 17 body keypoints (nose, eyes, shoulders, elbows, wrists, etc.)
```
Detects human body joints and draws a skeleton.
Useful for: fitness tracking, gesture recognition, sports analysis.

### Task 4: Image Classification
```python
model = YOLO("yolov8n-cls.pt")
results = model("image.jpg")
# → Single label for the entire image (e.g., "golden retriever")
```
No bounding boxes — just "what is this image?"
Useful for: sorting photos, quality control, content moderation.

### Task 5: Oriented Bounding Boxes (OBB)
```python
model = YOLO("yolov8n-obb.pt")
results = model("satellite.jpg")
# → Rotated bounding boxes (for aerial/satellite images)
```
Regular boxes are axis-aligned. OBB boxes can be rotated to fit
objects at any angle. Useful for: satellite imagery, document detection.

---

## 11. Real-World Project Ideas — Image Processing

### Project 1: People Counter
**Difficulty:** ★☆☆ Easy

Count how many people are in a room using a webcam.

```python
from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    results = model(frame, conf=0.5, verbose=False)

    # Count only "person" detections (class_id = 0)
    people = sum(1 for box in results[0].boxes if int(box.cls[0]) == 0)
    print(f"People in frame: {people}")

    cv2.imshow("People Counter", results[0].plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

**Use cases:**
- Occupancy monitoring in offices/classrooms
- Crowd counting at events
- Social distancing enforcement

---

### Project 2: Vehicle Counter on a Road
**Difficulty:** ★★☆ Medium

Count cars, trucks, and buses passing through a virtual line on a road.

```python
from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture("road_video.mp4")

# Vehicle class IDs in COCO: car=2, motorcycle=3, bus=5, truck=7
VEHICLE_CLASSES = {2, 3, 5, 7}
LINE_Y = 400  # Virtual counting line (y-coordinate)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.4, verbose=False)

    # Draw the counting line
    cv2.line(frame, (0, LINE_Y), (frame.shape[1], LINE_Y), (0, 0, 255), 2)

    for box in results[0].boxes:
        class_id = int(box.cls[0])
        if class_id in VEHICLE_CLASSES:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            center_y = (y1 + y2) // 2

            # Check if vehicle center crosses the line
            if abs(center_y - LINE_Y) < 10:
                print(f"Vehicle crossed! Class: {results[0].names[class_id]}")

    cv2.imshow("Vehicle Counter", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

**Use cases:**
- Traffic flow analysis
- Parking lot occupancy
- Toll booth automation

---

### Project 3: PPE (Safety Equipment) Detector
**Difficulty:** ★★★ Hard (requires custom training)

Detect whether workers are wearing helmets, vests, and gloves.

**Approach:**
1. Collect images of workers with/without PPE
2. Label them using [Roboflow](https://roboflow.com) or [CVAT](https://cvat.ai)
3. Train a custom YOLOv8 model (see Section 15)
4. Deploy on a camera at the construction site

**Use cases:**
- Construction site safety monitoring
- Factory compliance checking
- Automated safety audits

---

### Project 4: License Plate Reader
**Difficulty:** ★★★ Hard

Detect license plates, then use OCR to read the text.

```python
from ultralytics import YOLO
import cv2
# pip install easyocr
import easyocr

# Step 1: Detect license plates (custom-trained model)
plate_model = YOLO("license_plate_model.pt")

# Step 2: Read text from detected plates
reader = easyocr.Reader(['en'])

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    results = plate_model(frame, conf=0.5, verbose=False)

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        plate_crop = frame[y1:y2, x1:x2]

        # OCR on the cropped plate
        text = reader.readtext(plate_crop, detail=0)
        print(f"Plate: {''.join(text)}")

    cv2.imshow("Plate Reader", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 12. Real-World Project Ideas — Embedded & IoT

These projects combine YOLO with hardware like ESP8266, Raspberry Pi,
or Arduino — the intersection of computer vision and embedded systems.

### Project 5: YOLO + ESP8266 — Smart Light Control
**Difficulty:** ★★☆ Medium

Turn on a light when a person is detected, turn off when no one is around.

```
┌──────────┐    UDP     ┌──────────┐
│  Laptop  │ ────────── │ ESP8266  │
│  + YOLO  │  "on/off"  │  + LED   │
│  + Camera│            │          │
└──────────┘            └──────────┘

Logic:
  Person detected? → Send "on" via UDP → LED ON
  No person?       → Send "off" via UDP → LED OFF
```

```python
from ultralytics import YOLO
import cv2
import socket

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

# UDP socket to ESP8266 (same as p7/udp_client.py)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ESP_IP = "10.160.6.231"
ESP_PORT = 4210

last_state = None

while True:
    ret, frame = cap.read()
    results = model(frame, conf=0.5, verbose=False)

    # Check if any person is detected
    people = [b for b in results[0].boxes if int(b.cls[0]) == 0]

    if people and last_state != "on":
        sock.sendto(b"on", (ESP_IP, ESP_PORT))
        last_state = "on"
        print("Person detected → LED ON")
    elif not people and last_state != "off":
        sock.sendto(b"off", (ESP_IP, ESP_PORT))
        last_state = "off"
        print("No person → LED OFF")

    cv2.imshow("Smart Light", results[0].plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

**This is exactly how our workshop projects connect!**
- Camera + YOLO on your laptop
- UDP to ESP8266 (same protocol as `p7/udp_client.py`)
- ESP8266 controls the LED (same `p7.ino` firmware)

---

### Project 6: Intruder Alert System
**Difficulty:** ★★☆ Medium

Detect a person in a restricted area and trigger an alarm on ESP8266.

```
Camera watches a door/area
        │
        ▼
YOLO detects "person"
        │
        ▼
Is person in the "restricted zone"?
  (check if bounding box overlaps a defined region)
        │
    YES │         NO
        ▼          ▼
  Send "alarm"   Do nothing
  via UDP to
  ESP8266
        │
        ▼
  ESP8266 buzzer ON
  ESP8266 LED blinks
```

**Key concept — Region of Interest (ROI):**
```python
# Define a restricted zone (x1, y1, x2, y2)
RESTRICTED_ZONE = (100, 200, 500, 480)

for box in results[0].boxes:
    if int(box.cls[0]) == 0:  # person
        bx1, by1, bx2, by2 = map(int, box.xyxy[0])
        # Check overlap with restricted zone
        if (bx1 < RESTRICTED_ZONE[2] and bx2 > RESTRICTED_ZONE[0] and
            by1 < RESTRICTED_ZONE[3] and by2 > RESTRICTED_ZONE[1]):
            print("INTRUDER IN RESTRICTED ZONE!")
            sock.sendto(b"alarm", (ESP_IP, ESP_PORT))
```

---

### Project 7: Pet Feeder — Feed When Cat is Detected
**Difficulty:** ★★★ Hard

Automatically dispense food when a cat approaches the bowl.

```
Camera near food bowl
        │
        ▼
YOLO detects "cat" (class_id = 15)
        │
        ▼
Is cat near the bowl area?
        │
    YES │
        ▼
Send "feed" via UDP to ESP8266
        │
        ▼
ESP8266 activates servo motor
        │
        ▼
Food dispensed!
(Cooldown: don't feed again for 30 minutes)
```

---

### Project 8: Gesture-Controlled Robot via ROS2
**Difficulty:** ★★★ Hard

Use YOLO pose estimation to detect hand gestures and control a robot.

```
Camera → YOLOv8-pose → Detect hand position
                            │
                            ▼
                    Classify gesture:
                    - Hand up = "forward"
                    - Hand left = "turn left"
                    - Hand right = "turn right"
                    - Hand down = "stop"
                            │
                            ▼
                    Publish to ROS2 topic
                    /robot_command
                            │
                            ▼
                    Robot node subscribes
                    and moves accordingly
```

---

## 13. YOLO + ROS2 — Connecting Vision to Robots

In a real robot system, YOLO runs as a ROS2 node that publishes detections:

```
┌─────────────────┐     /camera/image      ┌──────────────────┐
│  Camera Node    │ ──────────────────────> │  YOLO Node       │
│  (publishes     │     (sensor_msgs/Image) │  (runs detection)│
│   raw frames)   │                         │                  │
└─────────────────┘                         └────────┬─────────┘
                                                     │
                                            /detections
                                            (custom msg)
                                                     │
                              ┌───────────────────────┼──────────────┐
                              │                       │              │
                              ▼                       ▼              ▼
                    ┌──────────────┐      ┌──────────────┐  ┌──────────────┐
                    │ Navigation   │      │ Safety Node  │  │ Display Node │
                    │ Node         │      │ (stop if     │  │ (show boxes  │
                    │ (avoid       │      │  person too  │  │  on screen)  │
                    │  obstacles)  │      │  close)      │  │              │
                    └──────────────┘      └──────────────┘  └──────────────┘
```

This is the same pub/sub pattern as our keyboard LED control system,
but with camera images and AI detections instead of keyboard commands!

---

## 14. YOLO + ESP8266 — Vision-Triggered Hardware

Here's how our workshop projects connect together:

```
┌─────────────────────────────────────────────────────────────────┐
│                     YOUR LAPTOP                                 │
│                                                                 │
│  ┌──────────┐    frame    ┌──────────┐   "on"/"off"            │
│  │  Webcam  │ ──────────> │  YOLO    │ ──────────────┐         │
│  └──────────┘             │  Model   │               │         │
│                           └──────────┘               │         │
│                                                      │ UDP     │
│                                                      │         │
└──────────────────────────────────────────────────────┼─────────┘
                                                       │
                                              WiFi Network
                                                       │
┌──────────────────────────────────────────────────────┼─────────┐
│                     ESP8266                           │         │
│                                                      ▼         │
│  ┌──────────┐    GPIO    ┌──────────┐   UDP Packet             │
│  │   LED    │ <───────── │  p7.ino  │ <───────────             │
│  │  (or     │            │  (UDP    │                          │
│  │  relay,  │            │  server) │                          │
│  │  motor)  │            └──────────┘                          │
│  └──────────┘                                                  │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

**The key insight:** Your laptop does the heavy AI processing (YOLO),
and the tiny ESP8266 just controls the hardware. This is called
**edge-cloud architecture** — the "cloud" (your laptop) does the thinking,
the "edge" (ESP8266) does the acting.

---

## 15. Training Your Own YOLO Model

Want to detect objects that aren't in the 80 COCO classes?
(e.g., your specific product, a custom part, a specific animal breed)

We provide **everything you need locally** — no Roboflow, no cloud,
no internet required (after initial setup).

### Our Local Training Tools

| Tool | File | What it does |
|------|------|-------------|
| **Web Annotation Tool** | `annotate/app.py` | Browser-based annotation UI (Flask + TailwindCSS) |
| **CLI Annotation Tool** | `annotate_images.py` | Command-line annotation with OpenCV |
| **Training Script** | `train_custom_model.py` | Set up dataset, train, predict, export |
| **Jupyter Notebook** | `yolo_training_workflow.ipynb` | Interactive step-by-step training guide |

### Step 1: Collect Images

- Take 50-500 photos of your object from different angles
- Include different lighting conditions, backgrounds, and distances
- More images = better model (but 50 can work for a demo)
- You can capture images from webcam using the Jupyter notebook

### Step 2: Set Up Dataset Structure

```bash
# This creates the folder structure and data.yaml for you
uv run python YoloExamples/train_custom_model.py \
    --setup --classes helmet no_helmet
```

This creates:
```
YoloExamples/my_dataset/
├── data.yaml          ← Dataset configuration (auto-generated)
├── train/
│   ├── images/        ← Put 80% of your images here
│   └── labels/        ← Labels saved here by annotation tool
└── val/
    ├── images/        ← Put 20% of your images here
    └── labels/        ← Labels saved here by annotation tool
```

**data.yaml** (auto-generated):
```yaml
train: train/images
val: val/images

nc: 2                  # Number of classes
names: ['helmet', 'no_helmet']  # Class names
```

### Step 3: Annotate Images Locally

**Option A: Web UI** (recommended — browser-based, like Roboflow):
```bash
uv run python YoloExamples/annotate/app.py
# Opens at http://localhost:5000
```
- Browse to your images folder or upload images in the browser
- Add/manage classes directly in the UI
- Click & drag to draw bounding boxes, right-click to delete
- Keyboard shortcuts, auto-save, ZIP export
- See the built-in **Usage / Help** page for full instructions

![Annotator Home Page](Annotator%20Tool/1.png)

Select a folder, define classes, and start annotating:

![Annotator Folder Browser](Annotator%20Tool/2.png)

Draw bounding boxes on the canvas, manage classes in the sidebar:

![Annotator Annotation Page](Annotator%20Tool/3.png)

**Option B: OpenCV CLI tool** (lightweight, no browser needed):
```bash
uv run python YoloExamples/annotate_images.py \
    --images YoloExamples/my_dataset/train/images/ \
    --classes helmet no_helmet
```

**Option C: Jupyter Notebook** (interactive, with visualization):
```bash
uv run jupyter lab YoloExamples/yolo_training_workflow.ipynb
```

**Option D: Online tools** (if you prefer):
- **[Roboflow](https://roboflow.com)** — Free online tool
- **[CVAT](https://cvat.ai)** — Open source, self-hosted

All tools save labels in **YOLO format** (`.txt` files with
`<class_id> <cx> <cy> <width> <height>` normalized to 0-1).

#### Multi-Class Labeling

Each image's `.txt` label file can have **multiple lines** with
**different class IDs**. For example, if a photo shows one person
wearing a helmet and another without:

```
0 0.4807 0.2089 0.3461 0.3750    ← class 0 (helmet)
1 0.7200 0.5100 0.2000 0.3000    ← class 1 (no_helmet)
```

In the Annotator, just select a class, draw a box, switch class,
draw another box — all on the same image. Each box = one line.
An image can have zero boxes (negative example), one, or many.

### Step 4: Train

```bash
# Using our training script
uv run python YoloExamples/train_custom_model.py --train
```

Or in Python:
```python
from ultralytics import YOLO

# Start from the pre-trained nano model (transfer learning)
model = YOLO("yolov8n.pt")

# Train on your custom dataset
model.train(
    data="YoloExamples/my_dataset/data.yaml",
    epochs=50,          # Number of training rounds
    imgsz=640,          # Image size
    batch=16,           # Images per batch
    name="my_model"     # Output folder name
)
```

Or from the command line:
```bash
uv run yolo detect train \
    data=YoloExamples/my_dataset/data.yaml \
    model=yolov8n.pt epochs=50
```

Or use the **Jupyter notebook** for an interactive experience:
```bash
uv run jupyter lab YoloExamples/yolo_training_workflow.ipynb
```

### Step 5: Use Your Trained Model

```python
model = YOLO("runs/detect/my_model/weights/best.pt")
results = model("test_image.jpg")
```

```bash
# Test on webcam
uv run python YoloExamples/train_custom_model.py \
    --predict --source 0

# Test on an image
uv run python YoloExamples/train_custom_model.py \
    --predict --source path/to/image.jpg

# Validate on the dataset
uv run python YoloExamples/train_custom_model.py --validate

# Export to ONNX
uv run python YoloExamples/train_custom_model.py --export onnx
```

---

## 16. Running YOLO on Edge Devices

### Raspberry Pi 5

```bash
# Install Ultralytics on Raspberry Pi
pip install ultralytics

# Run detection (will be slower than laptop, ~200ms per frame for nano)
yolo detect predict model=yolov8n.pt source=0
```

### NVIDIA Jetson (Nano, Orin)

Export to TensorRT for 5-10x speedup:
```python
model = YOLO("yolov8n.pt")
model.export(format="engine")  # Creates yolov8n.engine

# Use the TensorRT model
fast_model = YOLO("yolov8n.engine")
results = fast_model("image.jpg")  # Much faster!
```

### ESP32-CAM

The ESP32-CAM has a camera but is too weak to run YOLO directly.
Instead, stream frames to your laptop:

```
ESP32-CAM → WiFi stream → Laptop (YOLO) → UDP command → ESP8266 (LED)
```

### Export Formats

```python
model = YOLO("yolov8n.pt")

model.export(format="onnx")       # Universal format (CPU)
model.export(format="engine")     # NVIDIA TensorRT (GPU)
model.export(format="coreml")     # Apple devices
model.export(format="tflite")     # Android / Raspberry Pi
model.export(format="openvino")   # Intel devices
```

---

## 17. Common Terminology Glossary

| Term | Meaning |
|------|---------|
| **Bounding Box** | Rectangle around a detected object (x1, y1, x2, y2) |
| **Class** | Category of object (e.g., "person", "car", "dog") |
| **Confidence** | How sure the model is about a detection (0.0 to 1.0) |
| **CNN** | Convolutional Neural Network — the type of AI model YOLO uses |
| **COCO** | Common Objects in Context — the dataset YOLO is trained on (80 classes) |
| **Epoch** | One complete pass through the training dataset |
| **FPS** | Frames Per Second — how many images processed per second |
| **GPU** | Graphics Processing Unit — hardware that accelerates AI (NVIDIA CUDA) |
| **Inference** | Running a trained model on new data (as opposed to training) |
| **IoU** | Intersection over Union — measures overlap between two boxes |
| **mAP** | mean Average Precision — standard accuracy metric for detection |
| **NMS** | Non-Maximum Suppression — removes duplicate overlapping detections |
| **Pre-trained** | Model already trained on a large dataset (ready to use) |
| **Transfer Learning** | Starting from a pre-trained model and fine-tuning on your data |
| **Weights** | The learned parameters of the neural network (stored in .pt files) |

---

## 18. Commands Quick Reference

### Run Object Detection

```bash
# Using our script (webcam)
uv run python YoloExamples/object_detection.py

# Using Ultralytics CLI (webcam)
uv run yolo detect predict model=yolov8n.pt source=0 show=True

# On a single image
uv run yolo detect predict model=yolov8n.pt source=path/to/image.jpg

# On a video file
uv run yolo detect predict model=yolov8n.pt source=path/to/video.mp4

# On a YouTube video (requires pafy/yt-dlp)
uv run yolo detect predict model=yolov8n.pt source="https://youtube.com/..."
```

### Run Other Tasks

```bash
# Segmentation
uv run yolo segment predict model=yolov8n-seg.pt source=0 show=True

# Pose estimation
uv run yolo pose predict model=yolov8n-pose.pt source=0 show=True

# Classification
uv run yolo classify predict model=yolov8n-cls.pt source=path/to/image.jpg
```

### Training

```bash
# Train on custom dataset
uv run yolo detect train data=dataset.yaml model=yolov8n.pt epochs=50

# Resume interrupted training
uv run yolo detect train resume model=runs/detect/train/weights/last.pt
```

### Model Info

```bash
# Show model architecture and parameters
uv run yolo info model=yolov8n.pt

# Validate model on a dataset
uv run yolo detect val model=yolov8n.pt data=coco.yaml
```

---

## 19. Troubleshooting

### "No module named 'ultralytics'"
```bash
cd ~/ROS_Workshop
uv sync
# or
uv add ultralytics
```

### "Could not open camera"
- Is another program using the camera? Close it first.
- Try a different camera index: change `CAMERA_INDEX = 1` in the script.
- On Linux, check permissions: `ls -la /dev/video*`

### YOLO is very slow (< 5 FPS)
- Use the Nano model: `yolov8n.pt` (not medium/large)
- Reduce input resolution: `model(frame, imgsz=320)`
- If you have an NVIDIA GPU, install CUDA:
  ```bash
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
  ```

### "CUDA out of memory"
- Use a smaller model (nano or small)
- Reduce batch size during training: `batch=8` or `batch=4`
- Reduce image size: `imgsz=320`

### Model downloads every time
- The model is cached in the current directory as `yolov8n.pt`
- Make sure you're running from the same directory each time
- Or specify the full path: `YOLO("/home/user/models/yolov8n.pt")`

### Detection is inaccurate
- Lower the confidence threshold: `conf=0.3` (more detections)
- Use a larger model: `yolov8s.pt` or `yolov8m.pt`
- Ensure good lighting and camera angle
- For custom objects, train your own model (Section 15)

---

## Further Reading

- **[Ultralytics Docs](https://docs.ultralytics.com)** — Official documentation
- **[YOLO Paper (2015)](https://arxiv.org/abs/1506.02640)** — Original research paper
- **[COCO Dataset](https://cocodataset.org)** — The dataset YOLO is trained on
- **[Roboflow](https://roboflow.com)** — Label images and train custom models
- **[Ultralytics HUB](https://hub.ultralytics.com)** — Train models in the cloud (free tier)
