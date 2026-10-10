import socket

ROBOT_IP = "172.20.10.2"
PORT = 1000

command = bytearray(21)
command[0] = 0x80
command[1] = 2
command[2] = 0

print("Connecting to", ROBOT_IP)

try:
    with socket.create_connection((ROBOT_IP, PORT), timeout=5) as sock:
        sock.settimeout(10)
        print("TCP connected")

        sock.sendall(command)
        print("Sent 21-byte zero-speed command")

        try:
            data = sock.recv(104)
            print("Received bytes:", len(data))
            print("Raw data:", data.hex(" "))

        except socket.timeout:
            print("TIMEOUT: No sensor data received within 10 seconds")

        # Best-effort stop command.
        sock.sendall(command)

except Exception as error:
    print("Connection or communication error:", error)

print("Diagnostic finished")
