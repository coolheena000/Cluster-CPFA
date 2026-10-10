import socket

ROBOT_IP = "172.20.10.2"
PORT = 1000

command = bytearray(21)
command[0] = 0x80
command[1] = 2
command[2] = 0

def receive_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Robot disconnected")
        data.extend(chunk)
    return data

print("Connecting to Robot 1...")

try:
    with socket.create_connection((ROBOT_IP, PORT), timeout=5) as sock:
        sock.settimeout(10)
        print("Connected to Robot 1")

        for i in range(5):
            sock.sendall(command)

            # GCtronic protocol: first read 1-byte header.
            header = receive_exact(sock, 1)
            print("Packet header:", header[0])

            if header[0] != 2:
                print("Unexpected packet header")
                break

            # Header 2 means sensor data follows.
            sensor = receive_exact(sock, 104)

            proximity = [
                sensor[37 + 2*j] + 256 * sensor[38 + 2*j]
                for j in range(8)
            ]

            print(f"Reading {i+1}: {proximity}")

        # Best-effort zero-speed command.
        sock.sendall(command)

except (socket.timeout, OSError, ConnectionError) as error:
    print("Communication error:", error)

print("Test finished")
