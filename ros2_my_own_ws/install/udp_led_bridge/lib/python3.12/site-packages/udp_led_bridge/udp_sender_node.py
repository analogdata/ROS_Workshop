import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UdpSenderNode(Node):
    def __init__(self):
        super().__init__('udp_sender_node')

        self.declare_parameter('esp_ip', '10.160.6.231')
        self.declare_parameter('esp_port', 4210)

        self.esp_ip = self.get_parameter('esp_ip').get_parameter_value().string_value
        self.esp_port = self.get_parameter('esp_port').get_parameter_value().integer_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2)

        self.subscription = self.create_subscription(
            String,
            'led_command',
            self.command_callback,
            10,
        )

        self.get_logger().info(
            f'UDP Sender ready. Sending to {self.esp_ip}:{self.esp_port}'
        )
        self.get_logger().info('Listening on topic: /led_command (publish "on" or "off")')

    def command_callback(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return

        self.get_logger().info(f'Sending: {cmd}')
        self.sock.sendto(cmd.encode(), (self.esp_ip, self.esp_port))

        try:
            data, _ = self.sock.recvfrom(1024)
            self.get_logger().info(f'ESP Response: {data.decode()}')
        except socket.timeout:
            self.get_logger().warn('No response from ESP (timeout)')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
