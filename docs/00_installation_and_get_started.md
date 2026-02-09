# Installation & Getting Started

This guide walks you through installing **everything** you need for the
ROS Workshop — from scratch, on Ubuntu Linux.

---

## Table of Contents

1. [Git — Version Control](#1-git--version-control)
2. [Clone This Repository](#2-clone-this-repository)
3. [uv — Python Package Manager](#3-uv--python-package-manager)
4. [Arduino IDE + ESP8266 Board + PubSubClient](#4-arduino-ide--esp8266-board--pubsubclient)
5. [ROS2 Jazzy](#5-ros2-jazzy)
6. [Docker](#6-docker)
7. [Webots — Robot Simulator](#7-webots--robot-simulator)
8. [OpenCV — Computer Vision](#8-opencv--computer-vision)
9. [Ultralytics (YOLOv8) — Object Detection](#9-ultralytics-yolov8--object-detection)
10. [AI Model Files (MediaPipe & dlib)](#10-ai-model-files-mediapipe--dlib)
11. [Verify Everything Works](#11-verify-everything-works)

---

## 1. Git — Version Control

### What is Git?
Git is a tool that tracks changes to your code. Think of it like "undo history"
for your entire project. It also lets multiple people work on the same code
without overwriting each other's work.

### Key Terms
- **Repository (repo)** — A project folder tracked by Git
- **Clone** — Download a copy of a repository
- **Commit** — Save a snapshot of your changes
- **Push** — Upload your commits to GitHub/GitLab
- **Pull** — Download the latest changes from GitHub/GitLab
- **Branch** — A separate line of development (like a parallel universe)

### Install Git

```bash
sudo apt update
sudo apt install git -y
```

### Configure Git (first time only)

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Essential Git Commands

```bash
# Check if Git is installed
git --version

# Clone a repository (download it)
git clone https://github.com/username/repo.git

# Check status (what files changed?)
git status

# Stage all changes for commit
git add .

# Commit with a message
git commit -m "Describe what you changed"

# Push to GitHub
git push

# Pull latest changes
git pull
```

---

## 2. Clone This Repository

```bash
# Go to your home directory (or wherever you want the project)
cd ~

# Clone the workshop repository
git clone https://github.com/YOUR_USERNAME/ROS_Workshop.git

# Enter the project directory
cd ROS_Workshop
```

> **Note:** Replace `YOUR_USERNAME` with the actual GitHub username or use
> the URL provided by the workshop instructor.

---

## 3. uv — Python Package Manager

### What is uv?
`uv` is a modern, super-fast Python package manager (like `pip`, but 10-100x faster).
It manages Python versions, virtual environments, and dependencies all in one tool.

### Why uv instead of pip?
- **Speed** — Installs packages in seconds, not minutes
- **Reproducibility** — Lock files ensure everyone gets the exact same versions
- **Virtual environments** — Automatically creates and manages `.venv`
- **Python versions** — Can install Python itself if needed

### Install uv

```bash
# Install uv (official installer)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Restart your terminal or run:
source ~/.bashrc    # or source ~/.zshrc if you use zsh
```

### Verify Installation

```bash
uv --version
```

### How to Use uv in This Project

```bash
cd ~/ROS_Workshop

# Install all dependencies (reads pyproject.toml, creates .venv)
uv sync

# Run a Python script using the project's virtual environment
uv run python p7/camera_test.py

# Add a new package
uv add package_name

# Remove a package
uv remove package_name
```

### What is pyproject.toml?
It's the project's configuration file. It lists:
- Project name and version
- Python version required
- All package dependencies

```toml
[project]
name = "ros-workshop"
requires-python = ">=3.13"
dependencies = [
    "mediapipe",
    "opencv-python",
    "dlib",
    "scipy",
    ...
]
```

### What is .venv?
A **virtual environment** — an isolated folder containing Python and all
installed packages. This keeps your project's packages separate from your
system Python, so different projects don't conflict with each other.

---

## 4. Arduino IDE + ESP8266 Board + PubSubClient

### What is Arduino IDE?
Arduino IDE is the program you use to write code for microcontrollers
(like ESP8266, Arduino Uno, etc.) and upload it to the board via USB.

### Install Arduino IDE

**Option A — From Ubuntu Software Center:**
Search for "Arduino IDE" and install.

**Option B — From terminal (recommended, gets latest version):**
```bash
# Download Arduino IDE 2.x AppImage
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.4_Linux_64bit.AppImage

# Make it executable
chmod +x arduino-ide_2.3.4_Linux_64bit.AppImage

# Run it
./arduino-ide_2.3.4_Linux_64bit.AppImage
```

**Option C — Using snap:**
```bash
sudo snap install arduino
```

### Add ESP8266 Board Support

The ESP8266 is NOT an official Arduino board, so we need to add it:

1. Open Arduino IDE
2. Go to **File → Preferences**
3. In **"Additional Board Manager URLs"**, paste:
   ```
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
4. Click **OK**
5. Go to **Tools → Board → Board Manager**
6. Search for **"esp8266"**
7. Click **Install** on "esp8266 by ESP8266 Community"
8. Wait for download (~150 MB)

### Select the ESP8266 Board

1. Go to **Tools → Board → esp8266**
2. Select **"NodeMCU 1.0 (ESP-12E Module)"**

### Install PubSubClient Library (MQTT)

PubSubClient is a library for MQTT communication (useful for IoT projects):

1. Go to **Sketch → Include Library → Manage Libraries**
2. Search for **"PubSubClient"**
3. Find **"PubSubClient by Nick O'Leary"**
4. Click **Install**

### USB Permissions (Linux)

If Arduino IDE can't find your ESP8266 on the USB port:

```bash
# Add your user to the dialout group (for USB serial access)
sudo usermod -a -G dialout $USER

# Log out and log back in (or reboot) for the change to take effect
```

### Test the Setup

1. Connect ESP8266 via USB
2. Go to **Tools → Port** → Select the port (e.g., `/dev/ttyUSB0`)
3. Open **File → Examples → 01.Basics → Blink**
4. Click **Upload** (→ arrow)
5. The LED on the ESP8266 should start blinking!

---

## 5. ROS2 Jazzy

### What is ROS2?
ROS2 (Robot Operating System 2) is a framework for building robot software.
It provides tools for communication between programs (nodes), sensor handling,
simulation, and more. See [docs/02_what_is_ros2.md](02_what_is_ros2.md) for a
beginner-friendly explanation.

### System Requirements
- **Ubuntu 24.04 LTS (Noble Numbat)** — ROS2 Jazzy's target platform
- At least 2 GB free disk space

### Install ROS2 Jazzy

```bash
# 1. Set locale (ensure UTF-8)
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add the ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. Install ROS2 Jazzy (Desktop = full install with GUI tools)
sudo apt update
sudo apt install ros-jazzy-desktop -y

# 6. Install development tools (colcon, rosdep, etc.)
sudo apt install ros-dev-tools -y
```

### Source ROS2 (IMPORTANT — do this in every terminal)

```bash
source /opt/ros/jazzy/setup.bash
```

**To make it automatic** (so you don't have to type it every time):
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify ROS2 Installation

```bash
# Check ROS2 version
ros2 --help

# Run a test: start a talker in one terminal
ros2 run demo_nodes_cpp talker

# In another terminal, start a listener
ros2 run demo_nodes_cpp listener
```

If the listener prints messages from the talker, ROS2 is working!

### Install colcon (Build Tool)

`colcon` is the build tool for ROS2 workspaces (like `make` for ROS2):

```bash
sudo apt install python3-colcon-common-extensions -y
```

### Install rosdep (Dependency Resolver)

`rosdep` automatically installs system dependencies for ROS2 packages:

```bash
sudo apt install python3-rosdep2 -y
sudo rosdep init
rosdep update
```

---

## 6. Docker

### What is Docker?
Docker lets you run applications in **containers** — lightweight, isolated
environments that include everything the app needs. Think of it as a
"mini virtual machine" but much faster and lighter.

### Why Docker for ROS2?
- Run ROS2 on any OS (even Windows/Mac) inside a container
- Avoid "it works on my machine" problems
- Easy to share exact environments with teammates

### Install Docker

```bash
# 1. Remove old versions (if any)
sudo apt remove docker docker-engine docker.io containerd runc 2>/dev/null

# 2. Install prerequisites
sudo apt update
sudo apt install ca-certificates curl gnupg -y

# 3. Add Docker's official GPG key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 4. Add Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 5. Install Docker
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

# 6. Add your user to the docker group (so you don't need sudo)
sudo usermod -aG docker $USER

# 7. Log out and log back in, then verify:
docker --version
docker run hello-world
```

### Useful Docker Commands

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# Pull a ROS2 image
docker pull ros:jazzy

# Run a ROS2 container interactively
docker run -it ros:jazzy bash

# Stop a container
docker stop <container_id>
```

---

## 7. Webots — Robot Simulator

### What is Webots?
Webots is a free, open-source 3D robot simulator. You can design robots,
create environments, and test your ROS2 code in simulation before running
it on real hardware.

### Install Webots

**Option A — From apt (recommended for ROS2 integration):**
```bash
sudo apt install ros-jazzy-webots-ros2 -y
```

**Option B — Standalone (latest version):**
```bash
# Download from the official website
# https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux

sudo snap install webots
```

### Verify Webots

```bash
# Launch Webots
webots &

# Or test with ROS2 integration
ros2 launch webots_ros2_epuck robot_launch.py
```

---

## 8. OpenCV — Computer Vision

### What is OpenCV?
OpenCV (Open Source Computer Vision) is a library for image and video
processing. It can capture camera frames, detect faces, draw shapes,
and much more.

### Install OpenCV

OpenCV is installed automatically when you run `uv sync` in this project
(it's listed in `pyproject.toml`). But if you need it system-wide:

```bash
# Via uv (project-level, recommended)
cd ~/ROS_Workshop
uv sync    # Installs opencv-python and opencv-contrib-python

# Via pip (system-wide, if needed)
pip install opencv-python opencv-contrib-python
```

### Verify OpenCV

```bash
cd ~/ROS_Workshop
uv run python -c "import cv2; print('OpenCV version:', cv2.__version__)"
```

### Test Camera with OpenCV

```bash
uv run python p7/camera_test.py
```

A window should open showing your webcam feed. Press `q` to quit.

---

## 9. Ultralytics (YOLOv8) — Object Detection

### What is Ultralytics / YOLO?
YOLO (You Only Look Once) is a real-time object detection AI model.
Ultralytics is the company that makes YOLOv8, the latest version.
It can detect objects, people, cars, etc. in images and video.

### Install Ultralytics

```bash
# Add to the project
cd ~/ROS_Workshop
uv add ultralytics

# Or install system-wide
pip install ultralytics
```

### Verify Ultralytics

```bash
uv run python -c "from ultralytics import YOLO; print('Ultralytics installed!')"
```

### Quick Test (detect objects in an image)

```python
from ultralytics import YOLO

# Load a pre-trained model (downloads automatically on first run)
model = YOLO("yolov8n.pt")  # 'n' = nano (smallest, fastest)

# Run detection on an image
results = model("https://ultralytics.com/images/bus.jpg")

# Show results
results[0].show()
```

> **Note:** Ultralytics is optional for this workshop. It's useful for
> advanced projects like detecting objects and sending commands to the ESP8266.

---

## 10. AI Model Files (MediaPipe & dlib)

These model files are too large for Git. Download them manually:

### MediaPipe Hand Landmarker (~12 MB)

Needed by `p7/finger_udp.py` for finger counting:

```bash
cd ~/ROS_Workshop
wget -O p7/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

### dlib Shape Predictor 68 (~99 MB)

Needed by `p7/drowsiness_udp.py` for drowsiness detection:

```bash
cd ~/ROS_Workshop
wget -O p7/shape_predictor_68_face_landmarks.dat.bz2 \
  https://github.com/davisking/dlib-models/raw/master/shape_predictor_68_face_landmarks.dat.bz2

# Decompress (the .bz2 file is compressed)
bunzip2 p7/shape_predictor_68_face_landmarks.dat.bz2
```

### YOLOv8 Nano Model (~6 MB)

Needed by `YoloExamples/object_detection.py` for real-time object detection:

```bash
cd ~/ROS_Workshop/YoloExamples
uv run python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

> **Note:** The model also auto-downloads the first time you run
> `object_detection.py`, but this pre-downloads it so you're ready offline.

### Verify Model Files Exist

```bash
ls -lh p7/hand_landmarker.task
# Should show ~12 MB

ls -lh p7/shape_predictor_68_face_landmarks.dat
# Should show ~99 MB

ls -lh YoloExamples/yolov8n.pt
# Should show ~6.2 MB
```

---

## 11. Verify Everything Works

Run these commands to check that all tools are installed correctly:

```bash
# Git
git --version
# Expected: git version 2.x.x

# uv
uv --version
# Expected: uv 0.x.x

# Python (via uv)
uv run python --version
# Expected: Python 3.13.x

# ROS2
ros2 --help
# Expected: usage: ros2 [-h] ...

# colcon
colcon --help
# Expected: usage: colcon ...

# Arduino IDE
# Open it from your applications menu or run the AppImage

# Docker
docker --version
# Expected: Docker version 2x.x.x

# OpenCV
uv run python -c "import cv2; print(cv2.__version__)"
# Expected: 4.x.x

# MediaPipe
uv run python -c "import mediapipe; print(mediapipe.__version__)"
# Expected: 0.10.x

# dlib
uv run python -c "import dlib; print(dlib.__version__)"
# Expected: 19.x.x

# Ultralytics (YOLO)
uv run python -c "import ultralytics; print(ultralytics.__version__)"
# Expected: 8.x.x
```

### Quick Sanity Test

```bash
cd ~/ROS_Workshop

# 1. Install Python dependencies
uv sync

# 2. Test camera
uv run python p7/camera_test.py
# (press 'q' to quit)

# 3. Build ROS2 keyboard control package
cd ros2_keyboar_led_control
colcon build --packages-select keyboard_node
source install/setup.bash

# 4. Test that the node starts (Ctrl+C to stop)
ros2 run keyboard_node status_display_node
```

If all of the above work, you're ready for the workshop!

---

## Summary Checklist

| Tool | What it's for | Install command |
|------|--------------|-----------------|
| **Git** | Version control | `sudo apt install git` |
| **uv** | Python package manager | `curl -LsSf https://astral.sh/uv/install.sh \| sh` |
| **Arduino IDE** | Flash ESP8266 | `sudo snap install arduino` |
| **ESP8266 Board** | Arduino board support | Arduino IDE → Board Manager → esp8266 |
| **PubSubClient** | MQTT library | Arduino IDE → Library Manager → PubSubClient |
| **ROS2 Jazzy** | Robot framework | `sudo apt install ros-jazzy-desktop` |
| **colcon** | ROS2 build tool | `sudo apt install python3-colcon-common-extensions` |
| **Docker** | Containers | `sudo apt install docker-ce` |
| **Webots** | Robot simulator | `sudo apt install ros-jazzy-webots-ros2` |
| **OpenCV** | Computer vision | `uv sync` (auto-installed) |
| **Ultralytics** | Object detection (YOLO) | `uv add ultralytics` |
| **MediaPipe** | Hand detection | `uv sync` (auto-installed) |
| **dlib** | Face landmarks | `uv sync` (auto-installed) |

---

## Need Help?

Join the workshop WhatsApp group for any support - Community Maintained:

👉 **[Click here to join](https://chat.whatsapp.com/GDMHZ3MlXunLEsWuaagXEY?mode=gi_t)**

Or scan the QR code in the README.
