"""
Simple UDP Client for ESP8266 LED Control
==========================================
This script sends text commands ("on" / "off") over UDP to an ESP8266
microcontroller on the same WiFi network. The ESP receives the command
and turns its built-in LED on or off.

What is UDP?
    UDP (User Datagram Protocol) is a way to send small packets of data
    over a network. It's like sending a letter - you write the address,
    drop it in the mailbox, and hope it arrives. It's fast but there's
    no guarantee the message will be delivered (unlike TCP, which is
    like a phone call where both sides confirm they heard each other).

How this works:
    [This Python Script] --UDP packet "on"--> [ESP8266] --> LED turns ON
    [This Python Script] <--UDP packet "LED ON"-- [ESP8266] (acknowledgment)
"""

# 'socket' is Python's built-in library for network communication
import socket

# --- Configuration ---
# The ESP8266's IP address on the WiFi network (shown on its Serial Monitor)
DEFAULT_IP = "10.160.6.231"
# The UDP port number the ESP8266 is listening on (must match the Arduino code)
ESP_PORT = 4210

# Ask the user for the ESP's IP, or use the default if they just press Enter
ip_input = input(f"Enter ESP8266 IP address [{DEFAULT_IP}]: ").strip()
ESP_IP = ip_input if ip_input else DEFAULT_IP

# --- Create a UDP Socket ---
# AF_INET    = We're using IPv4 addresses (like 192.168.1.1)
# SOCK_DGRAM = We want UDP (datagram = small packet of data)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# If we don't get a response within 2 seconds, stop waiting
sock.settimeout(2)

print(f"UDP Client ready. Sending to {ESP_IP}:{ESP_PORT}")
print("Commands: 'on' = LED ON, 'off' = LED OFF, 'quit' = exit")

# --- Main Loop: Read user input and send commands ---
while True:
    msg = input(">> ").strip()

    # Type 'quit' to exit the program
    if msg.lower() == "quit":
        print("Exiting.")
        break

    # Skip empty input (user just pressed Enter)
    if not msg:
        continue

    # Send the message as bytes to the ESP8266's IP and port
    # .encode() converts the string "on" into bytes b"on" (networks send bytes)
    sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))

    # Wait for a response from the ESP8266 (it sends back "LED ON" / "LED OFF")
    try:
        data, addr = sock.recvfrom(1024)  # Read up to 1024 bytes
        print(f"Response: {data.decode()}")  # .decode() converts bytes back to string
    except socket.timeout:
        print("No response (timeout)")

# Close the socket when done (free up the network resource)
sock.close()
