"""
UDP Sender Node — Bridges ROS2 Topic to ESP8266 via UDP
========================================================
This ROS2 node subscribes to the /led_command topic and forwards
every message it receives as a UDP packet to the ESP8266 microcontroller.
It also publishes the ESP's response (or error) to the /led_status topic.

How it fits in the system:
    ┌──────────────────┐    /led_command    ┌──────────────────┐    /led_status    ┌──────────────────┐
    │ keyboard_input   │ ──────────────────>│ udp_sender_node  │ ────────────────>│ status_display   │
    │                  │    "on" / "off"    │ (this node)      │   "LED ON" etc.  │                  │
    └──────────────────┘                    └────────┬─────────┘                  └──────────────────┘
                                                     │
                                                UDP  │  "on" / "off"
                                                     ▼
                                            ┌──────────────────┐
                                            │    ESP8266       │
                                            │    LED ON/OFF    │
                                            └──────────────────┘

What are ROS2 Parameters?
    Parameters are like settings for a node that you can change at launch time
    WITHOUT editing the code. For example, if the ESP's IP address changes,
    you just pass a different parameter value instead of modifying the script.

    Usage:
        ros2 run keyboard_node udp_sender_node --ros-args -p esp_ip:="192.168.1.50"
"""

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UdpSenderNode(Node):
    """
    A ROS2 node that:
        1. Subscribes to /led_command (receives "on" / "off")
        2. Sends the command via UDP to the ESP8266
        3. Publishes the ESP's response to /led_status
    """

    def __init__(self):
        # Initialize the node with the name 'udp_sender_node'
        super().__init__('udp_sender_node')

        # --- Declare Parameters (configurable at launch) ---
        # 'esp_ip' = the ESP8266's IP address on the WiFi network
        # 'esp_port' = the UDP port the ESP8266 listens on (must match Arduino code)
        self.declare_parameter('esp_ip', '10.160.6.231')
        self.declare_parameter('esp_port', 4210)

        # Read the parameter values
        self.esp_ip = self.get_parameter('esp_ip').get_parameter_value().string_value
        self.esp_port = self.get_parameter('esp_port').get_parameter_value().integer_value

        # --- Create a UDP Socket ---
        # AF_INET = IPv4, SOCK_DGRAM = UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Short timeout: we don't want to block ROS2 for too long waiting for ESP
        self.sock.settimeout(0.5)

        # --- Create Subscriber (listens for commands) ---
        # Subscribes to /led_command topic
        # When a message arrives, command_callback() is called automatically
        self.subscription = self.create_subscription(
            String,              # Message type
            '/led_command',      # Topic name
            self.command_callback,  # Function to call when message arrives
            10                   # Queue size
        )

        # --- Create Publisher (sends status updates) ---
        # Publishes ESP responses to /led_status topic
        self.status_publisher = self.create_publisher(String, '/led_status', 10)

        # --- Add a parameter change callback ---
        # This allows changing esp_ip and esp_port at runtime
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.get_logger().info(
            f'UDP Sender ready. Target: {self.esp_ip}:{self.esp_port}'
        )
        self.get_logger().info('Subscribed to /led_command | Publishing to /led_status')

    def _on_parameter_change(self, params):
        """
        Called automatically when someone changes a parameter at runtime.

        This lets you change the ESP IP without restarting the node:
            ros2 param set /udp_sender_node esp_ip "192.168.1.100"
        """
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'esp_ip':
                self.esp_ip = param.value
                self.get_logger().info(f'ESP IP changed to: {self.esp_ip}')
            elif param.name == 'esp_port':
                self.esp_port = param.value
                self.get_logger().info(f'ESP Port changed to: {self.esp_port}')

        return SetParametersResult(successful=True)

    def command_callback(self, msg):
        """
        Called every time a message arrives on /led_command.

        This function:
            1. Reads the command from the message (e.g., "on")
            2. Sends it as a UDP packet to the ESP8266
            3. Waits for the ESP's response
            4. Publishes the response (or error) to /led_status
        """
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: "{command}"')

        # --- Send the command via UDP ---
        # .encode() converts the string to bytes (networks send bytes, not strings)
        self.sock.sendto(command.encode(), (self.esp_ip, self.esp_port))

        # --- Wait for ESP8266's response ---
        status_msg = String()
        try:
            # recvfrom() blocks until data arrives or timeout expires
            data, addr = self.sock.recvfrom(1024)
            response = data.decode()
            self.get_logger().info(f'ESP response: "{response}"')
            status_msg.data = response
        except socket.timeout:
            # No response within 0.5 seconds
            self.get_logger().warn('No response from ESP (timeout)')
            status_msg.data = f'TIMEOUT: No response for "{command}"'

        # --- Publish the status ---
        # This sends the ESP's response to /led_status for the status_display_node
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        """Clean up the UDP socket when the node is shutting down."""
        self.sock.close()
        self.get_logger().info('UDP socket closed.')
        super().destroy_node()


def main(args=None):
    """Entry point for the udp_sender_node."""
    rclpy.init(args=args)
    node = UdpSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by Ctrl+C')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
