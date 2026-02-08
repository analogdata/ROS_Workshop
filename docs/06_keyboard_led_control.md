# Keyboard LED Control — 3-Node ROS2 System

## What Is This?

A **ROS2 Jazzy** system that lets you control an ESP8266 LED using keyboard keys.
Instead of one big program, we split the work into **3 small programs (nodes)**
that talk to each other through ROS2 topics.

| Key | Action |
|-----|--------|
| `a` | Turn LED **ON** |
| `b` | Turn LED **OFF** |
| `q` | **Quit** the keyboard node |

---

## The 3 Nodes

### Node 1: `keyboard_input_node` — The Brain
**Job:** Read your keystrokes and decide what command to send.

- Reads single keys from the terminal (no Enter needed)
- When you press `a`, it publishes `"on"` to the `/led_command` topic
- When you press `b`, it publishes `"off"` to the `/led_command` topic
- When you press `q`, it shuts down

**File:** `keyboard_node/keyboard_input_node.py`

### Node 2: `udp_sender_node` — The Messenger
**Job:** Take commands from ROS2 and deliver them to the ESP8266 via UDP.

- Subscribes to `/led_command` topic (listens for commands)
- Sends each command as a UDP packet to the ESP8266
- Waits for the ESP's response (e.g., "LED ON")
- Publishes the response to `/led_status` topic

**File:** `keyboard_node/udp_sender_node.py`

### Node 3: `status_display_node` — The Reporter
**Job:** Show what's happening in a clean, formatted display.

- Subscribes to `/led_status` topic
- Prints each status update with a timestamp and counter
- Shows warnings for timeouts/errors

**File:** `keyboard_node/status_display_node.py`

---

## How They Talk to Each Other

```
  Terminal                  ROS2 Topics                    Network
  ────────                  ───────────                    ───────

  You press 'a'
       │
       ▼
┌──────────────────┐
│ keyboard_input   │
│ node             │
│                  │
│ Publishes "on"   │──────┐
└──────────────────┘      │
                          │  Topic: /led_command
                          │  Message: "on"
                          │
                          ▼
                   ┌──────────────────┐
                   │ udp_sender_node  │
                   │                  │
                   │ 1. Receives "on" │
                   │ 2. Sends UDP ────┼──────────────> ESP8266
                   │ 3. Gets response │<────────────── "LED ON"
                   │ 4. Publishes     │──────┐
                   └──────────────────┘      │
                                             │  Topic: /led_status
                                             │  Message: "LED ON"
                                             │
                                             ▼
                                      ┌──────────────────┐
                                      │ status_display   │
                                      │ node             │
                                      │                  │
                                      │ Prints:          │
                                      │ [#1] [14:30:05]  │
                                      │ ESP Status:      │
                                      │ LED ON           │
                                      └──────────────────┘
```

---

## Why 3 Nodes Instead of 1 Program?

The simple `udp_client.py` does everything in one file. So why split it into 3 nodes?

### The Restaurant Analogy

Think of a restaurant:
- **Waiter** (keyboard_input_node) → Takes your order
- **Chef** (udp_sender_node) → Cooks the food (sends UDP)
- **Display Board** (status_display_node) → Shows order status

You could have ONE person do all three jobs, but:
1. **Flexibility** — Want to replace the waiter with a touchscreen? Just swap that one node.
2. **Reusability** — The chef doesn't care WHO placed the order. Any node can publish to `/led_command`.
3. **Debugging** — If something breaks, you know exactly which node to check.
4. **Scalability** — Want to add a logging node? Just subscribe to `/led_status`. No code changes needed.

### Real-World Example
Later, you could replace `keyboard_input_node` with:
- A voice recognition node
- A gesture detection node
- A web interface node
- A sensor-triggered node

...and `udp_sender_node` and `status_display_node` would work exactly the same!

---

## ROS2 Topics Used

| Topic | Type | Publisher | Subscriber | Data |
|-------|------|-----------|------------|------|
| `/led_command` | `std_msgs/String` | keyboard_input_node | udp_sender_node | `"on"` or `"off"` |
| `/led_status` | `std_msgs/String` | udp_sender_node | status_display_node | `"LED ON"`, `"LED OFF"`, or timeout message |

---

## Build & Run Instructions

### Step 1: Build the Package

```bash
# Navigate to the workspace
cd ~/ROS_Workshop/ros2_keyboar_led_control

# Build only our package
colcon build --packages-select keyboard_node

# Source the workspace (MUST do this in every new terminal)
source install/setup.bash
```

**What does each command do?**
- `colcon build` — Compiles and installs the ROS2 package
- `--packages-select keyboard_node` — Only build our package (faster)
- `source install/setup.bash` — Tells the terminal where to find our nodes

### Step 2: Run All 3 Nodes (3 Terminals)

You need **3 separate terminal windows**. In EACH terminal, first run:
```bash
cd ~/ROS_Workshop/ros2_keyboar_led_control
source install/setup.bash
```

**Terminal 1 — Status Display (start this first):**
```bash
ros2 run keyboard_node status_display_node
```
You should see:
```
[INFO] [status_display_node]: Status Display Node started!
[INFO] [status_display_node]: Listening on /led_status for updates...
```

**Terminal 2 — UDP Sender:**
```bash
ros2 run keyboard_node udp_sender_node
```
You should see:
```
[INFO] [udp_sender_node]: UDP Sender ready. Target: 10.160.6.231:4210
[INFO] [udp_sender_node]: Subscribed to /led_command | Publishing to /led_status
```

**Terminal 3 — Keyboard Input (start this last):**
```bash
ros2 run keyboard_node keyboard_input_node
```
You should see:
```
[INFO] [keyboard_input_node]: Keyboard LED Controller started!
[INFO] [keyboard_input_node]: Press: a = LED ON | b = LED OFF | q = Quit
```

Now press `a` or `b` and watch all 3 terminals!

### Step 3: Change ESP IP Address (Optional)

**Option A — At launch time (parameter):**
```bash
ros2 run keyboard_node udp_sender_node --ros-args -p esp_ip:="192.168.1.50"
```

**Option B — While running (dynamic):**
```bash
# In a 4th terminal:
ros2 param set /udp_sender_node esp_ip "192.168.1.50"
```

**Option C — Change the port too:**
```bash
ros2 run keyboard_node udp_sender_node --ros-args -p esp_ip:="192.168.1.50" -p esp_port:=5000
```

---

## Useful ROS2 Commands for Debugging

```bash
# List all running nodes
ros2 node list

# Expected output:
#   /keyboard_input_node
#   /udp_sender_node
#   /status_display_node

# List all active topics
ros2 topic list

# Expected output:
#   /led_command
#   /led_status

# Watch messages on a topic in real-time
ros2 topic echo /led_command
ros2 topic echo /led_status

# See topic details (type, publishers, subscribers)
ros2 topic info /led_command -v

# See all parameters of the UDP sender node
ros2 param list /udp_sender_node

# Get current value of a parameter
ros2 param get /udp_sender_node esp_ip

# Visualize the node graph (if rqt is installed)
rqt_graph
```

---

## Troubleshooting

### "No response from ESP (timeout)"
- Is the ESP8266 powered on and connected to WiFi?
- Is the IP address correct? Check with `ros2 param get /udp_sender_node esp_ip`
- Are both devices on the same WiFi network?

### Keyboard node doesn't respond to key presses
- Make sure Terminal 3 (keyboard node) is the **focused/active** window
- The terminal must be in focus to capture keystrokes

### "Package not found" when running `ros2 run`
- Did you run `colcon build`?
- Did you run `source install/setup.bash` in the **same** terminal?
- You must source in **every new terminal window**

### Status node shows nothing
- Is the UDP sender node running?
- Check with `ros2 topic echo /led_status` to see if messages are flowing

---

## File Structure

```
ros2_keyboar_led_control/
├── src/
│   └── keyboard_node/                    # ROS2 Package
│       ├── package.xml                   # Package metadata & dependencies
│       ├── setup.py                      # Build config & entry points
│       ├── setup.cfg                     # Install paths
│       ├── resource/
│       │   └── keyboard_node             # Ament resource marker
│       └── keyboard_node/                # Python module
│           ├── __init__.py               # Package marker
│           ├── keyboard_input_node.py    # Node 1: Keyboard input
│           ├── udp_sender_node.py        # Node 2: UDP sender
│           └── status_display_node.py    # Node 3: Status display
├── build/                                # (generated by colcon build)
├── install/                              # (generated by colcon build)
└── log/                                  # (generated by colcon build)
```

---
---

# Deep Dive — How Everything Works Under the Hood

This section explains **every important concept** in detail so you truly
understand what's happening, not just how to run it.

---

## Where Does the LED Status Come From?

This is a critical question: **Does the status come from the ESP8266 hardware
or is it generated locally on your computer?**

**Answer: The status comes from the real ESP8266 hardware.**

Here's the exact journey of a single key press:

```
Step 1: You press 'a' on your keyboard
        │
        ▼
Step 2: keyboard_input_node reads the key
        Creates a ROS2 String message: "on"
        Publishes it to /led_command topic
        │
        ▼
Step 3: udp_sender_node receives "on" from /led_command
        Converts "on" to bytes: b"on"
        Sends UDP packet to ESP8266 at 10.160.6.231:4210
        │
        ▼  (travels over WiFi)
        │
Step 4: ESP8266 receives the UDP packet
        Reads the message: "on"
        Physically turns the LED ON (digitalWrite LOW)
        Creates a reply: "LED ON"
        Sends "LED ON" back via UDP to your computer
        │
        ▼  (travels back over WiFi)
        │
Step 5: udp_sender_node receives "LED ON" from ESP8266
        Creates a ROS2 String message: "LED ON"
        Publishes it to /led_status topic
        │
        ▼
Step 6: status_display_node receives "LED ON" from /led_status
        Prints: [#1] [14:30:05] ESP Status: LED ON
```

### What if the ESP8266 is OFF or unreachable?

```
Step 3: udp_sender_node sends UDP "on" to ESP8266
        │
        ▼  (packet goes into the void...)
        │
Step 4: udp_sender_node waits 0.5 seconds... no response
        Creates message: "TIMEOUT: No response for 'on'"
        Publishes it to /led_status
        │
        ▼
Step 5: status_display_node prints:
        ⚠ TIMEOUT: No response for "on"
```

This means the status is **always truthful**:
- If it says "LED ON" → the ESP8266 **confirmed** it turned the LED on
- If it says "TIMEOUT" → the command **did not reach** the ESP (or it crashed)
- You never get a false "LED ON" when the ESP is actually off

---

## Deep Dive: keyboard_input_node.py

### The Threading Problem

ROS2's `rclpy.spin()` runs an infinite loop waiting for messages. But we ALSO
need an infinite loop reading keyboard input. Two infinite loops can't run in
the same thread!

**Solution: Use a background thread for keyboard reading.**

```
Main Thread                    Background Thread
───────────                    ─────────────────
rclpy.spin()                   _read_keyboard()
  │                              │
  ├── Waits for callbacks        ├── Waits for key press
  ├── Processes messages         ├── Reads one character
  ├── Handles timers             ├── Publishes command
  └── Repeats...                 └── Repeats...
```

### How Raw Terminal Input Works

Normally when you type in a terminal:
```
You type: h e l l o [Enter]
Terminal sends: "hello\n"
```

The terminal **waits for Enter** before sending anything. This is called
"cooked mode" or "canonical mode".

But we want INSTANT key detection — read each key the moment it's pressed,
without waiting for Enter. This is called "cbreak mode":

```python
# Save current terminal settings (so we can restore them later)
old_settings = termios.tcgetattr(sys.stdin)

# Switch to cbreak mode (instant key reading)
tty.setcbreak(sys.stdin.fileno())

# Now sys.stdin.read(1) returns immediately when ANY key is pressed
key = sys.stdin.read(1)  # Returns 'a', 'b', 'q', etc.

# IMPORTANT: Restore original settings when done!
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
```

**Why restore settings?** If you don't, your terminal stays in raw mode after
the program exits. You won't see what you type, arrow keys won't work, and
the terminal becomes unusable. The `try/finally` block ensures restoration
even if the program crashes.

### What is a Publisher?

```python
self.publisher_ = self.create_publisher(String, '/led_command', 10)
```

This line creates a **publisher** — an object that can send messages to a topic.

- `String` — The message type (from `std_msgs.msg`). It has one field: `data`
- `'/led_command'` — The topic name. Any subscriber listening to this topic
  will receive our messages
- `10` — Queue size. If the subscriber is slow, buffer up to 10 messages
  before dropping old ones

Publishing a message:
```python
msg = String()       # Create an empty String message
msg.data = "on"      # Set the data field
self.publisher_.publish(msg)  # Send it to /led_command
```

---

## Deep Dive: udp_sender_node.py

### ROS2 Parameters — Settings Without Code Changes

Parameters let you configure a node at launch time:

```python
# Declare parameters with default values
self.declare_parameter('esp_ip', '10.160.6.231')
self.declare_parameter('esp_port', 4210)

# Read the values
self.esp_ip = self.get_parameter('esp_ip').get_parameter_value().string_value
self.esp_port = self.get_parameter('esp_port').get_parameter_value().integer_value
```

**Three ways to change the IP:**

```bash
# Way 1: At launch time (--ros-args -p)
ros2 run keyboard_node udp_sender_node --ros-args -p esp_ip:="192.168.1.50"

# Way 2: While running (from another terminal)
ros2 param set /udp_sender_node esp_ip "192.168.1.50"

# Way 3: Check current value
ros2 param get /udp_sender_node esp_ip
```

Way 2 works because we registered a **parameter change callback**:
```python
self.add_on_set_parameters_callback(self._on_parameter_change)
```

This function is called automatically whenever someone changes a parameter,
and it updates `self.esp_ip` or `self.esp_port` in real-time.

### What is a Subscriber?

```python
self.subscription = self.create_subscription(
    String,                # Message type to expect
    '/led_command',        # Topic to listen on
    self.command_callback, # Function to call when a message arrives
    10                     # Queue size
)
```

This tells ROS2: "Whenever a `String` message appears on `/led_command`,
call `self.command_callback` with that message."

The callback function receives the message automatically:
```python
def command_callback(self, msg):
    command = msg.data  # "on" or "off"
    # ... send via UDP ...
```

### The UDP Send-and-Receive Cycle

```python
# 1. Send command to ESP8266
self.sock.sendto(command.encode(), (self.esp_ip, self.esp_port))

# 2. Wait for ESP's response
try:
    data, addr = self.sock.recvfrom(1024)  # Blocks up to 0.5 seconds
    response = data.decode()               # e.g., "LED ON"
    status_msg.data = response
except socket.timeout:
    status_msg.data = f'TIMEOUT: No response for "{command}"'

# 3. Publish status (whether success or timeout)
self.status_publisher.publish(status_msg)
```

The timeout of 0.5 seconds is important:
- **Too short** (e.g., 0.05s) → You might miss slow ESP responses
- **Too long** (e.g., 5s) → The node freezes for 5 seconds if ESP is offline
- **0.5 seconds** is a good balance for local WiFi

### This Node Has BOTH a Subscriber AND a Publisher

```
                    ┌──────────────────────────┐
  /led_command ───> │     udp_sender_node      │ ───> /led_status
  (subscriber)      │                          │      (publisher)
                    │  Also does UDP I/O       │
                    │  to the ESP8266          │
                    └──────────────────────────┘
```

This is very common in ROS2 — a node can subscribe to one topic, process the
data, and publish results to another topic. It's like a **pipeline**.

---

## Deep Dive: status_display_node.py

### The Simplest Node

This node only has a subscriber — no publishers, no UDP, no threads.
It just listens and prints.

```python
self.subscription = self.create_subscription(
    String, '/led_status', self.status_callback, 10
)
```

When a message arrives:
```python
def status_callback(self, msg):
    self.message_count += 1
    now = datetime.now().strftime('%H:%M:%S')

    if 'TIMEOUT' in msg.data:
        self.get_logger().warn(f'[#{self.message_count}] [{now}] ⚠ {msg.data}')
    else:
        self.get_logger().info(f'[#{self.message_count}] [{now}] ESP Status: {msg.data}')
```

### Why a Separate Node Just for Printing?

You might think: "Why not just print in `udp_sender_node`?"

Because **separation of concerns** is a core ROS2 principle:
- `udp_sender_node` does ONE thing: bridge ROS2 ↔ UDP
- `status_display_node` does ONE thing: display status

Tomorrow you could **replace** `status_display_node` with:
- A node that logs status to a CSV file
- A node that sends email/SMS alerts
- A node that shows a GUI dashboard
- A node that plays a sound when the LED changes

...and you wouldn't change a SINGLE line in `udp_sender_node`.

---

## Deep Dive: How ROS2 Jazzy Connects the Nodes

### The DDS Layer (What Happens Behind the Scenes)

When you run 3 nodes, they don't directly connect to each other. Instead,
ROS2 Jazzy uses a system called **DDS (Data Distribution Service)** under the hood:

```
┌─────────────────────────────────────────────────────────────┐
│                    DDS Middleware                            │
│                                                             │
│  keyboard_input_node ──publish──> /led_command ──deliver──> │
│                                                             │
│  udp_sender_node <──subscribe── /led_command                │
│  udp_sender_node ──publish──> /led_status ──deliver──>      │
│                                                             │
│  status_display_node <──subscribe── /led_status             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

DDS handles:
1. **Discovery** — Nodes automatically find each other (no IP addresses needed!)
2. **Matching** — Publishers and subscribers on the same topic are connected
3. **Delivery** — Messages are delivered reliably between nodes
4. **Serialization** — Converting Python objects to bytes and back

You don't need to configure any of this — ROS2 Jazzy does it all automatically.

### What is `rclpy.spin()`?

Every node calls `rclpy.spin(node)` at the end. Here's what it actually does:

```
rclpy.spin(node)
    │
    ▼
    ┌─── Infinite Loop ──────────────────────────┐
    │                                            │
    │  1. Check: any messages on subscribed      │
    │     topics?                                │
    │     ├── YES → call the callback function   │
    │     └── NO  → continue                     │
    │                                            │
    │  2. Check: any timer callbacks due?         │
    │     ├── YES → call the timer function      │
    │     └── NO  → continue                     │
    │                                            │
    │  3. Check: any parameter changes?           │
    │     ├── YES → call the parameter callback  │
    │     └── NO  → continue                     │
    │                                            │
    │  4. Check: shutdown requested?              │
    │     ├── YES → exit the loop                │
    │     └── NO  → go back to step 1            │
    │                                            │
    └────────────────────────────────────────────┘
```

Without `spin()`, the node would create itself and immediately exit!

### What Happens When You Press Ctrl+C?

```
Ctrl+C pressed
    │
    ▼
Python raises KeyboardInterrupt
    │
    ▼
except KeyboardInterrupt:
    node.get_logger().info('Interrupted')
    │
    ▼
finally:
    node.destroy_node()   # Clean up subscriptions, publishers, timers
    rclpy.shutdown()      # Shut down the ROS2 system
```

---

## Deep Dive: The Package Configuration Files

### package.xml — "Who Am I?"

```xml
<package format="3">
  <name>keyboard_node</name>           <!-- Package name -->
  <version>0.0.0</version>             <!-- Version number -->
  <description>...</description>        <!-- What this package does -->

  <depend>rclpy</depend>               <!-- We need the ROS2 Python library -->
  <depend>std_msgs</depend>            <!-- We need String message type -->

  <export>
    <build_type>ament_python</build_type>  <!-- This is a Python package -->
  </export>
</package>
```

This file tells ROS2:
- What this package is called
- What other packages it depends on
- How to build it (Python, not C++)

### setup.py — "How to Install Me"

The most important part is `entry_points`:

```python
entry_points={
    'console_scripts': [
        'keyboard_input_node = keyboard_node.keyboard_input_node:main',
        'udp_sender_node = keyboard_node.udp_sender_node:main',
        'status_display_node = keyboard_node.status_display_node:main',
    ],
},
```

Each line follows this format:
```
executable_name = python_module.file_name:function_name
```

So `keyboard_input_node = keyboard_node.keyboard_input_node:main` means:
- When you run `ros2 run keyboard_node keyboard_input_node`
- It finds the file `keyboard_node/keyboard_input_node.py`
- And calls the `main()` function inside it

### setup.cfg — "Where to Put My Scripts"

```ini
[install]
install_scripts=$base/lib/keyboard_node
```

This tells the installer to put the executable scripts in the ROS2 workspace's
`lib/keyboard_node/` directory, where `ros2 run` can find them.

---

## Complete Timeline: What Happens From Build to LED ON

```
1. You run: colcon build --packages-select keyboard_node
   └── colcon reads setup.py
   └── Creates executable scripts in install/lib/keyboard_node/
   └── Copies package.xml to install/share/keyboard_node/

2. You run: source install/setup.bash
   └── Adds install/ paths to your shell's PATH and PYTHONPATH
   └── Now your terminal knows where to find 'keyboard_node'

3. You run: ros2 run keyboard_node status_display_node
   └── ROS2 finds install/lib/keyboard_node/status_display_node
   └── Calls status_display_node.py → main()
   └── rclpy.init() → Connects to DDS
   └── Creates StatusDisplayNode → Subscribes to /led_status
   └── rclpy.spin() → Waiting for messages...

4. You run: ros2 run keyboard_node udp_sender_node
   └── Creates UdpSenderNode
   └── Declares parameters (esp_ip, esp_port)
   └── Opens UDP socket
   └── Subscribes to /led_command
   └── Creates publisher for /led_status
   └── DDS discovers status_display_node is listening on /led_status → connected!
   └── rclpy.spin() → Waiting for messages...

5. You run: ros2 run keyboard_node keyboard_input_node
   └── Creates KeyboardInputNode
   └── Creates publisher for /led_command
   └── DDS discovers udp_sender_node is listening on /led_command → connected!
   └── Starts keyboard thread
   └── rclpy.spin() → Waiting...

6. You press 'a'
   └── Keyboard thread reads 'a'
   └── Publishes String("on") to /led_command
   └── DDS delivers message to udp_sender_node

7. udp_sender_node.command_callback("on") fires
   └── Sends UDP b"on" to 10.160.6.231:4210
   └── ESP8266 receives it → turns LED ON → replies "LED ON"
   └── udp_sender_node receives "LED ON"
   └── Publishes String("LED ON") to /led_status
   └── DDS delivers message to status_display_node

8. status_display_node.status_callback("LED ON") fires
   └── Prints: [#1] [14:30:05] ESP Status: LED ON

Total time from key press to status display: ~5-50 milliseconds
(depending on WiFi latency to ESP8266)
```
