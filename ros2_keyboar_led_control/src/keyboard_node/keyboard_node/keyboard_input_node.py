"""
Keyboard Input Node — Reads Keystrokes and Publishes LED Commands
==================================================================
This ROS2 node captures single keystrokes from the terminal (without
needing to press Enter) and publishes them as commands to the /led_command topic.

Key Bindings:
    'a' → Publish "on"  (turn LED ON)
    'b' → Publish "off" (turn LED OFF)
    'q' → Quit the node gracefully

How it fits in the system:
    ┌──────────────────┐    /led_command    ┌──────────────────┐
    │ keyboard_input   │ ──────────────────>│ udp_sender_node  │
    │ (this node)      │    "on" / "off"    │ (sends to ESP)   │
    └──────────────────┘                    └──────────────────┘

What is "publishing" in ROS2?
    Publishing means sending a message to a "topic" (a named channel).
    Any other node that is "subscribed" to that topic will receive the message.
    It's like broadcasting on a radio channel — anyone tuned in can hear it.

What is `termios`?
    Normally, the terminal waits for you to press Enter before reading input.
    `termios` lets us change terminal settings so we can read ONE key at a time
    without waiting for Enter. This is called "raw mode" or "cbreak mode".
"""

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardInputNode(Node):
    """
    A ROS2 node that reads keyboard input and publishes LED commands.

    - Runs a background thread to read keystrokes (so ROS2 spin isn't blocked)
    - Publishes "on" or "off" to the /led_command topic
    - Quits cleanly when 'q' is pressed
    """

    def __init__(self):
        # Initialize the node with the name 'keyboard_input_node'
        super().__init__('keyboard_input_node')

        # --- Create a Publisher ---
        # This publisher sends String messages to the /led_command topic
        # Queue size 10 = buffer up to 10 messages if subscriber is slow
        self.publisher_ = self.create_publisher(String, '/led_command', 10)

        # Flag to signal the keyboard thread to stop
        self.running = True

        # Print instructions for the user
        self.get_logger().info('Keyboard LED Controller started!')
        self.get_logger().info('Press: a = LED ON | b = LED OFF | q = Quit')

        # --- Start keyboard reading in a separate thread ---
        # We need a thread because reading keyboard input is "blocking"
        # (it waits for a key press). If we did this in the main thread,
        # ROS2 spin() couldn't process callbacks at the same time.
        self.keyboard_thread = threading.Thread(target=self._read_keyboard, daemon=True)
        self.keyboard_thread.start()

    def _read_keyboard(self):
        """
        Reads single keystrokes from the terminal in a background thread.

        How raw terminal input works:
            1. Save the current terminal settings (so we can restore them later)
            2. Switch to "cbreak" mode (read one key at a time, no Enter needed)
            3. Read keys in a loop
            4. Restore original terminal settings when done
        """
        # Save the original terminal settings so we can restore them on exit
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            # Switch terminal to cbreak mode:
            # - Keys are available immediately (no Enter needed)
            # - Ctrl+C still works for interrupting
            tty.setcbreak(sys.stdin.fileno())

            while self.running:
                # Read exactly ONE character from stdin
                key = sys.stdin.read(1)

                if key == 'a':
                    # 'a' pressed → publish "on" command
                    self._publish_command('on')
                    self.get_logger().info('Key [a] → Publishing: "on" (LED ON)')

                elif key == 'b':
                    # 'b' pressed → publish "off" command
                    self._publish_command('off')
                    self.get_logger().info('Key [b] → Publishing: "off" (LED OFF)')

                elif key == 'q':
                    # 'q' pressed → quit
                    self.get_logger().info('Key [q] → Quitting...')
                    self.running = False
                    # Request ROS2 to shut down (this will stop rclpy.spin)
                    rclpy.shutdown()
                    break

                else:
                    # Any other key → ignore but inform the user
                    self.get_logger().info(
                        f'Key [{key}] not recognized. Use: a=ON, b=OFF, q=Quit'
                    )

        finally:
            # ALWAYS restore the original terminal settings
            # If we don't do this, the terminal stays in raw mode after exit
            # and behaves weirdly (no echo, no line editing, etc.)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _publish_command(self, command: str):
        """
        Create a String message and publish it to /led_command.

        Args:
            command: The command string ("on" or "off")
        """
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)


def main(args=None):
    """
    Entry point for the keyboard_input_node.

    ROS2 Node Lifecycle:
        1. rclpy.init()       → Initialize the ROS2 system
        2. Create the node    → Set up publishers, subscribers, etc.
        3. rclpy.spin()       → Keep the node alive, process callbacks
        4. Cleanup            → Destroy node and shut down ROS2
    """
    rclpy.init(args=args)
    node = KeyboardInputNode()

    try:
        # spin() keeps the node running and processing any callbacks
        # It will block here until rclpy.shutdown() is called (from 'q' key)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Interrupted by Ctrl+C')
    finally:
        # Clean up: only destroy if context is still valid
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
