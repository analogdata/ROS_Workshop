# ROS Workshop — ESP8266 LED Control via UDP

Control an LED on an ESP8266 microcontroller using UDP — from simple text commands
to AI-powered hand gestures, drowsiness detection, and a multi-node ROS2 keyboard
control system.

---

## Join the Workshop WhatsApp Group - Community Maintained

<p align="center">
  <a href="https://chat.whatsapp.com/GDMHZ3MlXunLEsWuaagXEY?mode=gi_t">
    <img src="docs/Whatsapp QR for Group/IISc-ROS.jpeg" alt="WhatsApp Group QR" width="250"/>
  </a>
</p>

<p align="center">
  <a href="https://chat.whatsapp.com/GDMHZ3MlXunLEsWuaagXEY?mode=gi_t">👉 Click here to join the WhatsApp Group</a>
</p>

---

## Project Structure

```
ROS_Workshop/
├── p7/                                  # ESP8266 + Python programs
│   ├── p7/p7.ino                        # Arduino: ESP8266 UDP LED server
│   ├── udp_client.py                    # Simple text-based UDP client
│   ├── camera_test.py                   # Basic OpenCV camera test
│   ├── finger_udp.py                    # Finger counting → LED control (MediaPipe)
│   ├── drowsiness_udp.py               # Drowsiness detection → LED alert (dlib)
│   ├── hand_landmarker.task             # [DOWNLOAD] MediaPipe hand model
│   └── shape_predictor_68_face_landmarks.dat  # [DOWNLOAD] dlib face model
│
├── ros2_my_own_ws/                      # ROS2 Workspace (single-node bridge)
│   └── src/udp_led_bridge/              # ROS2 package: UDP LED bridge
│       └── udp_led_bridge/
│           └── udp_sender_node.py       # ROS2 node: topic → UDP → ESP8266
│
├── ros2_keyboar_led_control/            # ROS2 Workspace (3-node keyboard system)
│   └── src/keyboard_node/              # ROS2 package: keyboard LED control
│       └── keyboard_node/
│           ├── keyboard_input_node.py   # Node 1: reads keystrokes (a/b/q)
│           ├── udp_sender_node.py       # Node 2: sends UDP to ESP8266
│           └── status_display_node.py   # Node 3: displays LED status
│
├── YoloExamples/                        # YOLO object detection examples
│   ├── object_detection.py            # Real-time webcam object detection
│   ├── annotate/                      # Web-based annotation tool (Flask)
│   │   ├── app.py                   # Flask backend + API routes
│   │   └── templates/               # Jinja2 + TailwindCSS pages
│   ├── annotate_images.py             # CLI annotation tool (OpenCV)
│   ├── train_custom_model.py          # Custom YOLO training pipeline
│   ├── yolo_training_workflow.ipynb   # Jupyter notebook: full training guide
│   └── yolov8n.pt                     # [AUTO-DOWNLOAD] YOLOv8 Nano model
│
├── docs/                                # Step-by-step documentation
│   ├── 00_installation_and_get_started.md  # Installation guide (Git, ROS2, Arduino, etc.)
│   ├── 01_what_is_udp.md               # UDP explained simply
│   ├── 02_what_is_ros2.md              # ROS2 concepts for beginners
│   ├── 03_project_overview.md          # Project structure & how things connect
│   ├── 04_build_and_run.md             # Full build & run instructions
│   ├── 05_how_each_program_works.md    # Deep dive into each program
│   ├── 06_keyboard_led_control.md      # 3-node keyboard system guide
│   ├── 07_yolo_ultralytics.md          # YOLO & Ultralytics deep dive
│   ├── 08_annotating_and_training.md   # Annotation, training & public datasets
│   ├── Annotator Tool/                 # Annotator tool screenshots
│   └── Whatsapp QR for Group/          # WhatsApp group QR code image
│
├── pyproject.toml                       # Python dependencies (managed by uv)
└── README.md                            # You are here!
```

---

## Quick Start

### 1. Flash the ESP8266

Open `p7/p7/p7.ino` in Arduino IDE, select your ESP8266 board, and upload.
Open Serial Monitor (115200 baud) to see the ESP's IP address.

### 2. Install Python Dependencies

```bash
uv sync
```

### 3. Download AI Model Files

These model files are too large for git. Download them into the `p7/` directory:

**MediaPipe Hand Landmarker** (~12 MB) — needed by `finger_udp.py`:
```bash
wget -O p7/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

**dlib Shape Predictor 68** (~99 MB) — needed by `drowsiness_udp.py`:
```bash
wget -O p7/shape_predictor_68_face_landmarks.dat.bz2 \
  https://github.com/davisking/dlib-models/raw/master/shape_predictor_68_face_landmarks.dat.bz2
bunzip2 p7/shape_predictor_68_face_landmarks.dat.bz2
```

**YOLOv8 Nano** (~6 MB) — needed by `YoloExamples/object_detection.py`:
```bash
cd YoloExamples
uv run python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```
> The model auto-downloads on first run too, but this pre-downloads it.

### 4. Run a Program

```bash
# Test camera
uv run python p7/camera_test.py

# Simple text control (type "on" / "off")
uv run python p7/udp_client.py

# Finger counting (2 fingers = ON, 3 = OFF)
uv run python p7/finger_udp.py

# Drowsiness detection (eyes closed = ON)
uv run python p7/drowsiness_udp.py

# YOLO real-time object detection (press q to quit, s to screenshot)
uv run python YoloExamples/object_detection.py
```

### 5. Run the ROS2 Single-Node Bridge

```bash
cd ros2_my_own_ws
colcon build --packages-select udp_led_bridge
source install/setup.bash

# Terminal 1: Start the node
ros2 run udp_led_bridge udp_sender_node

# Terminal 2: Send commands
ros2 topic pub --once /led_command std_msgs/String "data: 'on'"
ros2 topic pub --once /led_command std_msgs/String "data: 'off'"
```

### 6. Run the ROS2 Keyboard Control (3 Nodes)

```bash
cd ros2_keyboar_led_control
colcon build --packages-select keyboard_node
source install/setup.bash
```

Open **3 terminals** (run `cd ~/ROS_Workshop/ros2_keyboar_led_control && source install/setup.bash` in each):

```bash
# Terminal 1 — Status display
ros2 run keyboard_node status_display_node

# Terminal 2 — UDP sender
ros2 run keyboard_node udp_sender_node

# Terminal 3 — Keyboard input (press a=ON, b=OFF, q=Quit)
ros2 run keyboard_node keyboard_input_node
```

To use a different ESP IP:
```bash
ros2 run keyboard_node udp_sender_node --ros-args -p esp_ip:="192.168.1.50"
```

---

## Programs Overview

| Program | Method | What it does |
|---------|--------|-------------|
| `p7/p7/p7.ino` | Arduino | ESP8266 listens for UDP, controls LED |
| `p7/udp_client.py` | Terminal | Type "on"/"off" to control LED |
| `p7/camera_test.py` | Camera | Test webcam with OpenCV |
| `p7/finger_udp.py` | AI (MediaPipe) | Count fingers → control LED |
| `p7/drowsiness_udp.py` | AI (dlib) | Detect drowsiness → alert LED |
| `udp_sender_node.py` (udp_led_bridge) | ROS2 | Bridge ROS2 topic → UDP → ESP8266 |
| `keyboard_input_node.py` | ROS2 | Read keystrokes (a/b/q), publish commands |
| `udp_sender_node.py` (keyboard_node) | ROS2 | Forward commands via UDP, publish status |
| `status_display_node.py` | ROS2 | Display LED status updates in terminal |
| `YoloExamples/object_detection.py` | AI (YOLO) | Real-time object detection with webcam |
| `YoloExamples/annotate/app.py` | Tool (Flask) | Web-based image annotation (like Roboflow) |
| `YoloExamples/annotate_images.py` | Tool (CLI) | Command-line image annotation (OpenCV) |
| `YoloExamples/train_custom_model.py` | AI (YOLO) | Train/predict/export custom YOLO models |
| `YoloExamples/yolo_training_workflow.ipynb` | Notebook | Interactive Jupyter training guide |

---

## Annotator Tool (Web-Based Image Labeling)

Our browser-based annotation tool for labeling images in YOLO format — 100% local, no cloud needed.

```bash
uv run python YoloExamples/annotate/app.py
# Opens at http://localhost:5000
```

**Home Page** — Select a folder of images or upload new ones:

![Annotator Home Page](docs/Annotator%20Tool/1.png)

**Folder Browser** — Navigate to your images, define classes, and start annotating:

![Annotator Folder Browser](docs/Annotator%20Tool/2.png)

**Annotation Page** — Draw bounding boxes, manage classes, track progress, save & export:

![Annotator Annotation Page](docs/Annotator%20Tool/3.png)

Supports **multi-class labeling** — draw boxes with different classes on the same image.
Each box becomes a line in the YOLO `.txt` label file. See the
[Annotating & Training docs](docs/08_annotating_and_training.md) for full details.

---

## Network Setup

- **WiFi SSID:** OnePlusRajath
- **ESP8266 Default IP:** 10.160.6.231
- **UDP Port:** 4210
- Both your PC and ESP8266 must be on the **same WiFi network**

---

## Dependencies

**ROS2 Jazzy** is required for the ROS2 workspaces (`ros2_my_own_ws` and `ros2_keyboar_led_control`).

Python packages managed via `pyproject.toml` with [uv](https://docs.astral.sh/uv/):

- **opencv-python** / **opencv-contrib-python** — Camera and image processing
- **mediapipe** — Google's hand landmark detection
- **dlib** — Face landmark detection (68-point model)
- **scipy** — Distance calculations for EAR (Eye Aspect Ratio)
- **ultralytics** — YOLOv8 object detection
- **flask** — Web-based annotation tool
- **jupyterlab** — Jupyter notebooks for interactive training
- **matplotlib** — Plotting training results
- **requests** — HTTP library

---

## Documentation

See the `docs/` folder for beginner-friendly explanations:

0. **[Installation & Getting Started](docs/00_installation_and_get_started.md)** — Git, ROS2 Jazzy, Arduino, uv, Docker, Webots, etc.
1. **[What is UDP?](docs/01_what_is_udp.md)** — UDP vs TCP, sockets, ports
2. **[What is ROS2?](docs/02_what_is_ros2.md)** — Nodes, Topics, Messages, Parameters
3. **[Project Overview](docs/03_project_overview.md)** — How everything connects
4. **[Build & Run](docs/04_build_and_run.md)** — Step-by-step instructions + troubleshooting
5. **[How Each Program Works](docs/05_how_each_program_works.md)** — Deep dive with flowcharts
6. **[Keyboard LED Control](docs/06_keyboard_led_control.md)** — 3-node ROS2 system guide
7. **[YOLO & Ultralytics](docs/07_yolo_ultralytics.md)** — Object detection deep dive, project ideas, embedded + image processing
8. **[Annotating & Training](docs/08_annotating_and_training.md)** — Public datasets, local annotation, training pipeline, complete walkthroughs

---

## License

For educational/workshop use.