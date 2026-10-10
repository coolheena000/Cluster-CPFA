import socket
import struct
import time

ROBOT_IP = "172.20.10.2"
PORT = 1000

def motor_command(left, right):
    packet = bytearray(21)
    packet[0] = 0x80
    packet[1] = 2
    packet[2] = 0
    struct.pack_into("<hh", packet, 3, left, right)
    return packet

def receive_exact(sock, count):
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError("Robot disconnected")
        data.extend(chunk)
    return data

sock = None

try:
    sock = socket.create_connection((ROBOT_IP, PORT), timeout=5)
    sock.settimeout(3)
    print("Robot connected")

    sock.sendall(motor_command(0, 0))
    if receive_exact(sock, 1) != b'\x02':
        raise RuntimeError("Unexpected header")
    receive_exact(sock, 104)

    print("Moving slowly for 1 second")
    start = time.monotonic()

    while time.monotonic() - start < 1.0:
        sock.sendall(motor_command(100, 100))
        if receive_exact(sock, 1) != b'\x02':
            raise RuntimeError("Unexpected header")
        receive_exact(sock, 104)
        time.sleep(0.05)

    print("Motor test finished")

except Exception as error:
    print("Error:", error)

finally:
    if sock is not None:
        try:
            sock.sendall(motor_command(0, 0))
            print("STOP command sent")
        except OSError:
            print("STOP command failed. Switch robot OFF.")
        sock.close()
