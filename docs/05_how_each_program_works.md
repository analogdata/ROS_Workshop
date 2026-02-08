# How Each Program Works (Deep Dive)

## Program 1: ESP8266 Arduino Code (`p7/p7/p7.ino`)

### What is an ESP8266?
An ESP8266 is a tiny, cheap microcontroller (~$3) with built-in WiFi.
Think of it as a mini computer that can connect to the internet and
control things like LEDs, motors, and sensors.

### The Two Functions Every Arduino Program Has

```
setup()  ──> Runs ONCE when the board powers on
              (connect to WiFi, configure pins)

loop()   ──> Runs FOREVER in a loop after setup
              (check for UDP packets, control LED)
```

### Flow of the Program
```
Power ON
   │
   ▼
setup()
   ├── Set LED pin as output
   ├── Connect to WiFi "OnePlusRajath"
   ├── Print IP address to Serial Monitor
   └── Start listening on UDP port 4210
   │
   ▼
loop() (repeats forever)
   ├── Any UDP packet arrived?
   │   ├── NO  → do nothing, loop again
   │   └── YES → read the message
   │              ├── "on"  → LED ON  + reply "LED ON"
   │              ├── "off" → LED OFF + reply "LED OFF"
   │              └── other → reply "Unknown command"
   └── Go back to start of loop()
```

### Active LOW LED (Important!)
The ESP8266's built-in LED is wired "backwards":
- `digitalWrite(LOW)` = LED **ON** (current flows)
- `digitalWrite(HIGH)` = LED **OFF** (no current)

This is called "active low" and is very common in microcontrollers.

---

## Program 2: Simple UDP Client (`p7/udp_client.py`)

### Flow
```
Start
  │
  ▼
Ask for ESP IP address (or use default 10.160.6.231)
  │
  ▼
Create UDP socket
  │
  ▼
┌─── Loop ──────────────────────────────┐
│  Wait for user to type a command      │
│  │                                    │
│  ├── "quit" → exit                    │
│  ├── empty  → skip                    │
│  └── anything else:                   │
│       ├── Send it via UDP to ESP      │
│       ├── Wait for response (2 sec)   │
│       │   ├── Got response → print it │
│       │   └── Timeout → print error   │
│       └── Go back to top of loop      │
└───────────────────────────────────────┘
```

### Key Concepts
- **`socket.socket(AF_INET, SOCK_DGRAM)`** - Creates a UDP socket
- **`.sendto(data, (ip, port))`** - Sends bytes to a specific address
- **`.recvfrom(1024)`** - Waits to receive up to 1024 bytes
- **`.encode()` / `.decode()`** - Converts between strings and bytes

---

## Program 3: Camera Test (`p7/camera_test.py`)

### Flow
```
Start
  │
  ▼
Open camera (VideoCapture(0))
  │
  ├── Failed? → print error, exit
  │
  ▼
┌─── Loop ──────────────────────┐
│  Read one frame from camera   │
│  │                            │
│  ├── Failed? → break          │
│  │                            │
│  ▼                            │
│  Display frame in window      │
│  │                            │
│  ▼                            │
│  Check for 'q' key press      │
│  ├── Yes → break              │
│  └── No  → continue loop     │
└───────────────────────────────┘
  │
  ▼
Release camera + close windows
```

### What is a "Frame"?
A frame is a single image captured from the camera. It's stored as a
**NumPy array** (a grid of numbers). Each pixel has 3 values: Blue, Green, Red.

For a 640x480 camera:
- Frame shape: (480, 640, 3) = 480 rows × 640 columns × 3 colors
- That's 921,600 numbers for ONE frame!
- At 30 FPS, that's ~27 million numbers per second

---

## Program 4: Finger Counter (`p7/finger_udp.py`)

### How MediaPipe Detects Hands
MediaPipe uses a neural network (AI) that was trained on millions of hand images.
Given a camera frame, it outputs 21 "landmark" points on your hand:

```
        4 (thumb tip)
       /
      3
     /
    2          8 (index tip)    12 (middle tip)   16 (ring tip)    20 (pinky tip)
   /          /                /                  /                /
  1          7               11                 15               19
  |         /               /                  /                /
  |        6              10                 14               18
  |       /              /                  /                /
  0 ─── 5 ──────── 9 ──────────── 13 ──────────── 17
  (wrist)
```

### How Finger Counting Works

**For the thumb (special case):**
The thumb moves sideways, so we compare X coordinates:
- If tip (4) is to the LEFT of joint (3) → thumb is open

**For other fingers (index, middle, ring, pinky):**
We compare Y coordinates (remember: in images, Y increases DOWNWARD):
- If tip is ABOVE joint (smaller Y) → finger is open
- If tip is BELOW joint (larger Y) → finger is closed

```
Finger OPEN:          Finger CLOSED:
                      
  8 (tip, y=100)        6 (joint, y=200)
  |                     |
  7                     7
  |                     |
  6 (joint, y=300)      8 (tip, y=350)
  
  tip.y < joint.y       tip.y > joint.y
  → OPEN!               → CLOSED!
```

### Flow
```
Start
  │
  ▼
Setup: ESP IP, UDP socket, MediaPipe model, Camera
  │
  ▼
┌─── Loop ──────────────────────────────────────┐
│  Read frame → flip → convert to RGB           │
│  │                                            │
│  ▼                                            │
│  Run MediaPipe hand detection                 │
│  │                                            │
│  ├── No hand found → finger_count = 0         │
│  │                                            │
│  └── Hand found:                              │
│       ├── Draw landmarks + skeleton           │
│       ├── Count open fingers (0-5)            │
│       │                                       │
│       ▼                                       │
│  Display finger count on screen               │
│  │                                            │
│  ├── 2 fingers → command = "on"               │
│  ├── 3 fingers → command = "off"              │
│  └── other     → no command                   │
│  │                                            │
│  ├── Command changed? → Send UDP to ESP       │
│  │                                            │
│  ▼                                            │
│  Show frame, check for 'q' key                │
└───────────────────────────────────────────────┘
```

---

## Program 5: Drowsiness Detector (`p7/drowsiness_udp.py`)

### How It Detects Drowsiness

**Step 1: Find the face** using dlib's face detector (HOG-based)

**Step 2: Find 68 landmarks** on the face:
```
     Eyebrows: 17-26
        Eyes: 36-47        ← We use these!
        Nose: 27-35
       Mouth: 48-67
         Jaw: 0-16
```

**Step 3: Calculate Eye Aspect Ratio (EAR)**

The 6 points around each eye:
```
       p2 ---- p3
      /          \
  p1               p4
      \          /
       p6 ---- p5
```

EAR formula:
```
EAR = (|p2-p6| + |p3-p5|) / (2 × |p1-p4|)

     = (vertical height) / (horizontal width)
```

- **Eyes open:** EAR ≈ 0.25 to 0.35
- **Eyes closed:** EAR ≈ 0.05 to 0.15
- **Threshold:** We use 0.25

**Step 4: Count consecutive "closed" frames**
- If EAR < 0.25 for 20+ frames (~0.67 seconds) → DROWSY!
- If eyes open at any point → reset counter

### Flow
```
Start
  │
  ▼
Setup: ESP IP, UDP socket, dlib models, Camera
  │
  ▼
┌─── Loop ──────────────────────────────────────┐
│  Read frame → flip → convert to grayscale     │
│  │                                            │
│  ▼                                            │
│  Detect faces with dlib                       │
│  │                                            │
│  For each face:                               │
│  ├── Find 68 landmarks                        │
│  ├── Extract left eye (points 36-41)          │
│  ├── Extract right eye (points 42-47)         │
│  ├── Calculate EAR for both eyes              │
│  ├── Average the two EAR values               │
│  │                                            │
│  ├── EAR < 0.25?                              │
│  │   ├── YES → increment frame_counter        │
│  │   └── NO  → reset frame_counter to 0       │
│  │                                            │
│  ├── frame_counter >= 20?                     │
│  │   ├── YES → DROWSY! command = "on"         │
│  │   └── NO  → Awake, command = "off"         │
│  │                                            │
│  └── Command changed? → Send UDP to ESP       │
│  │                                            │
│  Show frame, check for 'q' key                │
└───────────────────────────────────────────────┘
```

---

## Program 6: ROS2 UDP Bridge (`udp_sender_node.py`)

### How It Fits in the ROS2 World

```
┌──────────────────────────────────────────────────────────────┐
│                      ROS2 System                             │
│                                                              │
│  ┌─────────────┐    /led_command    ┌──────────────────┐     │
│  │ Any Node    │ ──────────────────>│ udp_sender_node  │     │
│  │ (publisher) │    (String msg)    │                  │     │
│  └─────────────┘                    │ Receives msg     │     │
│                                     │ Sends via UDP    │─────┼──> ESP8266
│  Examples of publishers:            │                  │     │
│  - ros2 topic pub (terminal)        └──────────────────┘     │
│  - A camera node                                             │
│  - A sensor node                                             │
│  - Another robot                                             │
└──────────────────────────────────────────────────────────────┘
```

### Flow
```
Start
  │
  ▼
rclpy.init() → Initialize ROS2
  │
  ▼
Create UdpSenderNode
  ├── Declare parameters (esp_ip, esp_port)
  ├── Create UDP socket
  └── Subscribe to /led_command topic
  │
  ▼
rclpy.spin() → Wait for messages...
  │
  ▼ (message arrives on /led_command)
command_callback()
  ├── Read the message text (e.g., "on")
  ├── Send it via UDP to ESP8266
  ├── Wait for ESP response
  └── Log the result
  │
  ▼
Back to waiting (spin continues)...
  │
  ▼ (Ctrl+C pressed)
Cleanup: close socket, destroy node, shutdown ROS2
```

### Key Difference from Plain Python Scripts
The plain Python scripts (udp_client.py, finger_udp.py, etc.) are
**standalone programs** - they do everything themselves.

The ROS2 node is a **team player** - it only handles the UDP part.
Other nodes can publish commands to it. This makes it easy to swap
out the "brain" (what decides to send "on" or "off") without changing
the UDP code.
