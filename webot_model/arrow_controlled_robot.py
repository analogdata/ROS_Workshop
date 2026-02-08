"""
Keyboard-controlled 2-wheeled robot controller for Webots
Arrow keys:
  ↑ Forward
  ↓ Backward
  ← Turn Left
  → Turn Right
"""

from controller import Robot, Keyboard

# ---------------- CONSTANTS ----------------
BASE_SPEED = 4.0   # rad/s (forward/backward)
TURN_SPEED = 3.0   # rad/s (turning)

# ---------------- ROBOT INIT ----------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------------- KEYBOARD INIT ----------------
keyboard = Keyboard()
keyboard.enable(timestep)

# ---------------- MOTOR INIT ----------------
left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")

# ---- SAFETY CHECK (prevents NoneType error) ----
if left_motor is None or right_motor is None:
    print("❌ ERROR: Motors not found. Check motor names in .wbt file.")
    print("Expected motor names: left_motor, right_motor")
    exit(1)

# Velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

print("✅ Keyboard control active")
print("Use arrow keys to move the robot")

# ---------------- MAIN LOOP ----------------
while robot.step(timestep) != -1:
    key = keyboard.getKey()

    left_speed = 0.0
    right_speed = 0.0

    if key == Keyboard.UP:
        # Forward
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED

    elif key == Keyboard.DOWN:
        # Backward
        left_speed = -BASE_SPEED
        right_speed = -BASE_SPEED

    elif key == Keyboard.LEFT:
        # Turn left
        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED

    elif key == Keyboard.RIGHT:
        # Turn right
        
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED

    # Apply speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

