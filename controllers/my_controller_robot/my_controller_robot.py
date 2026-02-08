"""
Controller for a simple 2-wheeled robot with MCP server integration.
"""

import json
import os
import time
from pathlib import Path
from controller import Robot

# --- Paths for data exchange ---
robot_name = os.environ.get("WEBOTS_ROBOT_NAME")
if not robot_name:
    # Fallback if environment variable is not set (e.g. testing outside webots)
    print("⚠️ WEBOTS_ROBOT_NAME not set, defaulting to 'SimpleRobot'")
    robot_name = "SimpleRobot"

CONTROLLER_DIR = Path(__file__).parent
# Assuming the structure is webot_mcp/controllers/simple_chassis_controller/simple_chassis_controller.py
# We need to go up to webot_mcp/ to find the data directory
ROBOT_DIR = CONTROLLER_DIR.parent.parent
DATA_DIR = ROBOT_DIR / "data" / robot_name
COMMANDS_FILE = DATA_DIR / "commands.json"
STATUS_FILE = DATA_DIR / "status.json"

# Create the data directory
DATA_DIR.mkdir(parents=True, exist_ok=True)

# Clear the commands file on controller startup
if COMMANDS_FILE.exists():
    try:
        os.remove(COMMANDS_FILE)
        print("✅ Old commands file cleared on startup.")
    except OSError as e:
        print(f"❌ Error clearing commands file on startup: {e}")

# --- Robot initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Motor initialization ---
motors = {}
motor_names = ["left_motor", "right_motor"]

for name in motor_names:
    motor = robot.getDevice(name)
    if motor:
        motors[name] = motor
        # Set to velocity control mode
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
        print(f"✅ Motor '{name}' initialized.")
    else:
        print(f"❌ Motor '{name}' not found.")

# --- Global variables ---
last_command_time = 0

def process_commands():
    """Processes commands from the MCP server."""
    global last_command_time

    if not COMMANDS_FILE.exists():
        return

    try:
        with open(COMMANDS_FILE, 'r', encoding='utf-8') as f:
            command = json.load(f)

        command_time = command.get('timestamp', 0)
        if command_time <= last_command_time:
            return

        last_command_time = command_time
        action = command.get('action')
        
        print(f"📥 Received action: {action}")

        if action == "set_motor_velocity":
            motor_name = command.get('motor_name')
            velocity = command.get('velocity', 0.0)
            
            if motor_name in motors:
                # Clamp velocity to max speed (let's say 10.0 rad/s)
                max_speed = 10.0
                velocity = max(-max_speed, min(max_speed, velocity))
                
                motors[motor_name].setVelocity(velocity)
                print(f"✅ Set {motor_name} velocity to {velocity}")
            else:
                print(f"❌ Motor '{motor_name}' not found.")

    except json.JSONDecodeError:
        pass
    except Exception as e:
        print(f"❌ Error processing command: {e}")

def update_status():
    """Updates the status for the MCP server."""
    current_time = time.time()

    status_data = {
        "timestamp": current_time,
        "webots_connected": True,
        "robot_name": robot_name,
        "motors": list(motors.keys())
    }

    try:
        with open(STATUS_FILE, 'w', encoding='utf-8') as f:
            json.dump(status_data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"❌ Error writing status: {e}")

# --- Main loop ---
if __name__ == "__main__":
    print("🚀 Simple Robot controller started. Waiting for commands...")

    while robot.step(timestep) != -1:
        process_commands()
        update_status()

    print("🚪 Robot controller is shutting down.")