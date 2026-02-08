# Build & Run Instructions (Step by Step)

## Prerequisites

Before you start, make sure you have:
- **Python 3.13+** installed
- **uv** (Python package manager) installed
- **ROS2** (Humble or later) installed and sourced
- **Arduino IDE** with ESP8266 board support
- **ESP8266** (NodeMCU) microcontroller
- **Webcam** (for camera/finger/drowsiness programs)
- Both your PC and ESP8266 on the **same WiFi network** (OnePlusRajath)

---

## Step 1: Flash the ESP8266 (Arduino)

This step uploads the LED server code to the ESP8266 microcontroller.

### 1.1 Open Arduino IDE
- Open the file: `p7/p7/p7.ino`

### 1.2 Install ESP8266 Board Support (if not done)
- Go to **File → Preferences**
- In "Additional Board Manager URLs", add:
  ```
  http://arduino.esp8266.com/stable/package_esp8266com_index.json
  ```
- Go to **Tools → Board → Board Manager**
- Search for "ESP8266" and install it

### 1.3 Configure and Upload
- **Tools → Board** → Select "NodeMCU 1.0 (ESP-12E Module)"
- **Tools → Port** → Select the USB port (e.g., `/dev/ttyUSB0`)
- Click the **Upload** button (→ arrow)
- Wait for "Done uploading"

### 1.4 Find the ESP's IP Address
- Open **Tools → Serial Monitor**
- Set baud rate to **115200**
- Press the **RST** button on the ESP8266
- You should see:
  ```
  Connecting to WiFi...
  Connected! IP: 10.160.6.231
  Listening on UDP port 4210
  ```
- **Note down the IP address!** You'll need it for the Python scripts.

---

## Step 2: Install Python Dependencies

Open a terminal in the project root directory:

```bash
cd ~/ROS_Workshop
```

### 2.1 Install all packages
```bash
uv sync
```

This reads `pyproject.toml` and installs: `mediapipe`, `opencv-python`,
`opencv-contrib-python`, `dlib`, `scipy`, `requests`

### 2.2 Verify installation
```bash
uv run python -c "import cv2; print('OpenCV:', cv2.__version__)"
uv run python -c "import mediapipe; print('MediaPipe:', mediapipe.__version__)"
uv run python -c "import dlib; print('dlib:', dlib.__version__)"
```

---

## Step 3: Run the Python Programs

> **Important:** Always run from the project root (`~/ROS_Workshop`)
> so that `uv run` can find the virtual environment.

### 3a. Test the Camera First
```bash
uv run python p7/camera_test.py
```
- A window should open showing your webcam feed
- Press **q** to quit
- If this doesn't work, fix your camera before trying other programs

### 3b. Simple UDP Client (Text Commands)
```bash
uv run python p7/udp_client.py
```
- Press Enter to use the default IP, or type the ESP's IP
- Type `on` and press Enter → LED should turn ON
- Type `off` and press Enter → LED should turn OFF
- Type `quit` to exit

### 3c. Finger Counting LED Control
```bash
uv run python p7/finger_udp.py
```
- Press Enter to use the default IP
- Show **2 fingers** to the camera → LED turns ON
- Show **3 fingers** to the camera → LED turns OFF
- Press **q** to quit

### 3d. Drowsiness Detection LED Alert
```bash
uv run python p7/drowsiness_udp.py
```
- Press Enter to use the default IP
- Keep your eyes open → screen shows "Awake", LED is OFF
- Close your eyes for ~1 second → screen shows "DROWSY!", LED turns ON
- Press **q** to quit

---

## Step 4: Build and Run the ROS2 Node

### 4.1 Build the ROS2 Package

```bash
# Navigate to the ROS2 workspace
cd ~/ROS_Workshop/ros2_my_own_ws

# Build only our package
colcon build --packages-select udp_led_bridge

# Source the workspace (IMPORTANT: do this every time you open a new terminal)
source install/setup.bash
```

**What does each command do?**
- `colcon build` - Compiles/installs the ROS2 package (like "make" for ROS2)
- `source install/setup.bash` - Tells your terminal where to find the new package

### 4.2 Run the UDP Sender Node

**Terminal 1** - Start the node:
```bash
cd ~/ROS_Workshop/ros2_my_own_ws
source install/setup.bash
ros2 run udp_led_bridge udp_sender_node
```

You should see:
```
[INFO] [udp_sender_node]: UDP Sender ready. Sending to 10.160.6.231:4210
[INFO] [udp_sender_node]: Listening on topic: /led_command (publish "on" or "off")
```

### 4.3 Send Commands via ROS2 Topic

**Terminal 2** - Publish commands:
```bash
# Turn LED ON
ros2 topic pub --once /led_command std_msgs/String "data: 'on'"

# Turn LED OFF
ros2 topic pub --once /led_command std_msgs/String "data: 'off'"
```

**What does this command mean?**
- `ros2 topic pub` - Publish a message to a topic
- `--once` - Send it only once (without this, it keeps sending)
- `/led_command` - The topic name
- `std_msgs/String` - The message type
- `"data: 'on'"` - The actual message content

### 4.4 Override the ESP IP (Optional)

If your ESP has a different IP address:
```bash
ros2 run udp_led_bridge udp_sender_node --ros-args -p esp_ip:="192.168.1.50"
```

### 4.5 Useful ROS2 Debugging Commands

```bash
# List all running nodes
ros2 node list

# List all active topics
ros2 topic list

# See messages flowing through a topic (live)
ros2 topic echo /led_command

# See info about a topic (type, publishers, subscribers)
ros2 topic info /led_command

# See the parameters of a node
ros2 param list /udp_sender_node
```

---

## Troubleshooting

### "No response (timeout)" when sending UDP
- **Check WiFi**: Are both your PC and ESP8266 on the same network?
- **Check IP**: Is the ESP's IP address correct? Check Serial Monitor.
- **Check Port**: Both sides must use port 4210.
- **Firewall**: Your PC's firewall might be blocking UDP. Try disabling it temporarily.

### Camera not opening
- Make sure no other program is using the camera
- Try a different camera index: change `VideoCapture(0)` to `VideoCapture(1)`
- On Linux, check permissions: `ls -la /dev/video*`

### MediaPipe "model not found" error
- Make sure `hand_landmarker.task` is in the `p7/` directory
- Download it:
  ```bash
  wget https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task -O p7/hand_landmarker.task
  ```

### dlib "shape predictor not found" error
- Make sure `shape_predictor_68_face_landmarks.dat` is in the `p7/` directory
- Download it:
  ```bash
  wget https://github.com/davisking/dlib-models/raw/master/shape_predictor_68_face_landmarks.dat.bz2 -O p7/shape_predictor_68_face_landmarks.dat.bz2
  bunzip2 p7/shape_predictor_68_face_landmarks.dat.bz2
  ```

### ROS2 "package not found" error
- Did you run `colcon build`?
- Did you run `source install/setup.bash` in the SAME terminal?
- You need to source it in EVERY new terminal window.

### `uv sync` fails or is slow
- Check your internet connection
- If a stray `pyproject.toml` exists in your home directory (`~/pyproject.toml`),
  delete it — it creates a uv workspace that interferes with the project.
