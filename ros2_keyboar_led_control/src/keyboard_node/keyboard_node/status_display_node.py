"""
Status Display Node — Shows LED Status Updates in the Terminal
===============================================================
This ROS2 node subscribes to the /led_status topic and displays
every status update in the terminal with clear formatting.

How it fits in the system:
    ┌──────────────────┐    /led_command    ┌──────────────────┐    /led_status    ┌──────────────────┐
    │ keyboard_input   │ ──────────────────>│ udp_sender_node  │ ────────────────>│ status_display   │
    │                  │                    │                  │                  │ (this node)      │
    └──────────────────┘                    └──────────────────┘                  └──────────────────┘

Why a separate status node?
    In ROS2, we follow the "single responsibility" principle:
    - keyboard_input_node → ONLY reads keyboard input
    - udp_sender_node     → ONLY sends UDP and gets responses
    - status_display_node → ONLY displays status

    This means you could REPLACE the status_display_node with, say, a
    node that logs to a file, sends an email alert, or shows a GUI —
    all without changing the other nodes. That's the power of ROS2!

What is a Subscriber?
    A subscriber "listens" to a topic. Whenever a new message is published
    to that topic, the subscriber's callback function is called automatically.
    Think of it like subscribing to a YouTube channel — you get notified
    whenever a new video (message) is uploaded (published).
"""

from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusDisplayNode(Node):
    """
    A ROS2 node that subscribes to /led_status and prints status updates.

    It shows:
        - Timestamp of each status update
        - The status message from the ESP8266
        - A running count of how many updates have been received
    """

    def __init__(self):
        # Initialize the node with the name 'status_display_node'
        super().__init__('status_display_node')

        # Counter to track how many status messages we've received
        self.message_count = 0

        # --- Create a Subscriber ---
        # Subscribes to /led_status topic
        # When a message arrives, status_callback() is called automatically
        self.subscription = self.create_subscription(
            String,                # Message type (std_msgs/String)
            '/led_status',         # Topic name to listen on
            self.status_callback,  # Function to call when a message arrives
            10                     # Queue size (buffer up to 10 messages)
        )

        self.get_logger().info('Status Display Node started!')
        self.get_logger().info('Listening on /led_status for updates...')
        self.get_logger().info('─' * 50)

    def status_callback(self, msg):
        """
        Called every time a new status message arrives on /led_status.

        Displays the status with a timestamp and message counter.
        """
        self.message_count += 1

        # Get current time for the timestamp
        now = datetime.now().strftime('%H:%M:%S')

        # Determine if this is a success or error/timeout
        status_text = msg.data
        if 'TIMEOUT' in status_text:
            # Timeout or error — show warning
            self.get_logger().warn(
                f'[#{self.message_count}] [{now}] ⚠ {status_text}'
            )
        else:
            # Success — show the ESP's response
            self.get_logger().info(
                f'[#{self.message_count}] [{now}] ESP Status: {status_text}'
            )


def main(args=None):
    """Entry point for the status_display_node."""
    rclpy.init(args=args)
    node = StatusDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Status display stopped.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
