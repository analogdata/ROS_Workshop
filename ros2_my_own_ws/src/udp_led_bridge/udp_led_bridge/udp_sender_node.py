"""
UDP Sender Node for ROS2
========================
This is a ROS2 node that acts as a bridge between ROS2 topics and a UDP device
(ESP8266 microcontroller). When someone publishes a message like "on" or "off"
to the /led_command topic, this node sends that command over UDP to the ESP8266,
which then turns the LED on or off.

Think of it like this:
    [ROS2 Topic: /led_command] --> [This Node] --> [UDP over WiFi] --> [ESP8266 LED]
"""

# Python's built-in library for network communication (sending/receiving data)
import socket

# rclpy = ROS Client Library for Python
# This is the main library that lets us write ROS2 programs in Python
import rclpy

# Node is the base class for all ROS2 nodes
# A "node" is like a small program that does one specific job in a robot system
from rclpy.node import Node

# String is a message type from ROS2's standard message library
# It's just a simple text message (like sending "on" or "off")
from std_msgs.msg import String


class UdpSenderNode(Node):
    """
    A ROS2 Node that listens for string messages on a topic
    and forwards them as UDP packets to an ESP8266 microcontroller.
    """

    def __init__(self):
        # Initialize the parent Node class with the name 'udp_sender_node'
        # This name is how other nodes and tools (like ros2 node list) identify us
        super().__init__('udp_sender_node')

        # --- ROS2 Parameters ---
        # Parameters are like settings that can be changed when you start the node
        # without modifying the code. Think of them as configurable variables.
        # Example: ros2 run udp_led_bridge udp_sender_node --ros-args -p esp_ip:="192.168.1.50"
        self.declare_parameter('esp_ip', '10.160.6.231')   # ESP8266's IP address
        self.declare_parameter('esp_port', 4210)            # UDP port the ESP listens on

        # Read the actual parameter values (could be default or user-provided)
        self.esp_ip = self.get_parameter('esp_ip').get_parameter_value().string_value
        self.esp_port = self.get_parameter('esp_port').get_parameter_value().integer_value

        # --- UDP Socket Setup ---
        # Create a UDP socket (SOCK_DGRAM = UDP, as opposed to SOCK_STREAM = TCP)
        # UDP is like sending a postcard - fast, no connection needed, but no guarantee of delivery
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2)  # Wait max 2 seconds for a response from ESP

        # --- ROS2 Subscription ---
        # A subscription listens for messages on a "topic"
        # Topics are like radio channels - anyone can broadcast, anyone can listen
        # Here we listen on the 'led_command' topic for String messages
        # When a message arrives, it calls self.command_callback
        # The '10' is the queue size (how many messages to buffer)
        self.subscription = self.create_subscription(
            String,             # Message type we expect
            'led_command',      # Topic name to listen on
            self.command_callback,  # Function to call when a message arrives
            10,                 # Queue size
        )

        # Log a message so we know the node started successfully
        # get_logger() gives us ROS2's logging system (shows up in terminal with timestamps)
        self.get_logger().info(
            f'UDP Sender ready. Sending to {self.esp_ip}:{self.esp_port}'
        )
        self.get_logger().info('Listening on topic: /led_command (publish "on" or "off")')

    def command_callback(self, msg):
        """
        This function is called automatically every time a new message
        arrives on the /led_command topic.

        msg.data contains the actual string text (e.g., "on" or "off")
        """
        cmd = msg.data.strip()
        if not cmd:
            return

        # Log what we're sending
        self.get_logger().info(f'Sending: {cmd}')

        # Send the command as a UDP packet to the ESP8266
        # .encode() converts the string to bytes (networks send bytes, not strings)
        self.sock.sendto(cmd.encode(), (self.esp_ip, self.esp_port))

        # Try to receive an acknowledgment from the ESP8266
        try:
            data, _ = self.sock.recvfrom(1024)  # Read up to 1024 bytes
            self.get_logger().info(f'ESP Response: {data.decode()}')
        except socket.timeout:
            self.get_logger().warn('No response from ESP (timeout)')

    def destroy_node(self):
        """Clean up: close the UDP socket when the node shuts down."""
        self.sock.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point for the node.

    The typical ROS2 Python node lifecycle:
    1. rclpy.init()       - Initialize the ROS2 system
    2. Create the node    - Our UdpSenderNode
    3. rclpy.spin()       - Keep the node running, processing callbacks
    4. Cleanup            - Destroy node and shutdown ROS2
    """
    # Step 1: Initialize ROS2 communication
    rclpy.init(args=args)

    # Step 2: Create our node
    node = UdpSenderNode()

    try:
        # Step 3: Keep the node alive and processing incoming messages
        # spin() is like an infinite loop that waits for messages and calls callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C was pressed - that's okay, just exit gracefully
        pass
    finally:
        # Step 4: Clean up
        node.destroy_node()
        rclpy.shutdown()


# This allows the file to be run directly with: python3 udp_sender_node.py
# But normally in ROS2, we use: ros2 run udp_led_bridge udp_sender_node
if __name__ == '__main__':
    main()
