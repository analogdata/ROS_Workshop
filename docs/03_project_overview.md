# Project Overview

## What Does This Project Do?

This project controls an LED on an ESP8266 microcontroller using different
methods - from a simple text command to AI-powered hand gesture and drowsiness
detection. Everything communicates over WiFi using UDP.

---

## Project Structure

```
ROS_Workshop/
│
├── p7/                              # All ESP8266 + Python programs
│   ├── p7/
│   │   └── p7.ino                   # Arduino code for ESP8266 (UDP LED server)
│   │
│   ├── udp_client.py                # Simple text-based UDP client
│   ├── camera_test.py               # Basic camera test (OpenCV)
│   ├── finger_udp.py                # Finger counting → LED control (MediaPipe)
│   ├── drowsiness_udp.py            # Drowsiness detection → LED alert (dlib)
│   │
│   ├── hand_landmarker.task         # MediaPipe hand detection model (~12MB)
│   └── shape_predictor_68_face_landmarks.dat  # dlib face landmark model (~99MB)
│
├── ros2_my_own_ws/                  # ROS2 Workspace
│   └── src/
│       └── udp_led_bridge/          # ROS2 Package
│           ├── package.xml          # Package metadata and dependencies
│           ├── setup.py             # Build configuration + entry points
│           ├── setup.cfg            # Install paths
│           └── udp_led_bridge/      # Python module (the actual code)
│               ├── __init__.py      # Makes this directory a Python package
│               └── udp_sender_node.py  # ROS2 node that sends UDP commands
│
├── docs/                            # You are here! Documentation
├── pyproject.toml                   # Python dependencies (managed by uv)
└── main.py                          # Main entry point (if needed)
```

---

## The 5 Programs Explained

### 1. ESP8266 Arduino Code (`p7/p7/p7.ino`)
**What:** Runs on the ESP8266 microcontroller
**Does:** Connects to WiFi, listens for UDP packets, controls the LED
**Language:** C++ (Arduino)

### 2. Simple UDP Client (`p7/udp_client.py`)
**What:** A terminal-based program
**Does:** You type "on" or "off", it sends the command to the ESP8266
**Good for:** Testing that the ESP8266 and UDP connection work

### 3. Camera Test (`p7/camera_test.py`)
**What:** Opens your webcam and shows the video feed
**Does:** Just displays what the camera sees
**Good for:** Testing that your camera works with OpenCV

### 4. Finger Counter (`p7/finger_udp.py`)
**What:** AI-powered hand gesture control
**Does:** Counts your fingers using MediaPipe AI, sends commands based on count
**How:** 2 fingers = LED ON, 3 fingers = LED OFF
**Uses:** MediaPipe Hand Landmarker (Google's AI model)

### 5. Drowsiness Detector (`p7/drowsiness_udp.py`)
**What:** AI-powered drowsiness alert system
**Does:** Watches your eyes, detects if you're falling asleep
**How:** Eyes closed for too long = LED ON (alert!), eyes open = LED OFF
**Uses:** dlib face landmarks + Eye Aspect Ratio (EAR) algorithm

### 6. ROS2 UDP Bridge (`ros2_my_own_ws/.../udp_sender_node.py`)
**What:** A ROS2 node that bridges ROS2 topics to UDP
**Does:** Listens on `/led_command` topic, forwards messages to ESP8266 via UDP
**Good for:** Integrating LED control into a larger ROS2 robot system

---

## How Everything Connects

```
                        Same WiFi Network (OnePlusRajath)
                    ┌─────────────────────────────────────────┐
                    │                                         │
  ┌─────────────────┴───────────────────┐    ┌───────────────┴──┐
  │          Your Computer              │    │    ESP8266       │
  │                                     │    │                  │
  │  Choose ONE of these to send:       │    │  Listens on      │
  │                                     │    │  UDP port 4210   │
  │  1. udp_client.py (type commands)   │    │                  │
  │  2. finger_udp.py (show fingers)    │──> │  Receives "on"   │
  │  3. drowsiness_udp.py (close eyes)  │    │  or "off"        │
  │  4. ROS2 udp_sender_node            │    │                  │
  │                                     │    │  Controls LED    │
  └─────────────────────────────────────┘    └──────────────────┘
```
