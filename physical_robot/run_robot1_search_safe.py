"""Physical e-puck2 search prototype with improved STOP handling.
Not a complete Carlos CPFA port. Always keep hardware power switch accessible.
"""
import argparse
import csv
import random
import socket
import struct
import time
from pathlib import Path

PORT = 1000

def packet(left=0, right=0):
    data = bytearray(21)
    data[:3] = b'\x80\x02\x00'
    struct.pack_into('<hh', data, 3, left, right)
    return data

def exact(sock, n):
    data = bytearray()
    while len(data) < n:
        part = sock.recv(n - len(data))
        if not part:
            raise ConnectionError('Robot disconnected')
        data.extend(part)
    return data

def exchange(sock, left, right):
    sock.sendall(packet(left, right))
    if exact(sock, 1) != b'\x02':
        raise RuntimeError('Unexpected response header')
    raw = exact(sock, 104)
    return [raw[37 + 2*i] + 256 * raw[38 + 2*i] for i in range(8)]

def stop_robot(sock, ip):
    # Use the same packet as the user's independently successful manual STOP.
    # A separate TCP session is attempted because the existing session did not
    # reliably stop the wheels in earlier physical tests.
    if sock is not None:
        try:
            sock.settimeout(1.0)
            sock.sendall(packet(0, 0))
            print('STOP sent on existing connection')
        except OSError as exc:
            print('Existing-connection STOP failed:', exc)
        finally:
            try:
                sock.close()
            except OSError:
                pass
    for attempt in range(1, 4):
        try:
            with socket.create_connection((ip, PORT), timeout=2.0) as fresh:
                fresh.settimeout(1.0)
                fresh.sendall(packet(0, 0))
            print(f'STOP sent on fresh connection (attempt {attempt})')
        except OSError as exc:
            print(f'Fresh-connection STOP attempt {attempt} failed: {exc}')
        time.sleep(0.15)
    print('VERIFY WHEELS ARE STOPPED. If moving, SWITCH ROBOT OFF.')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='172.20.10.2')
    ap.add_argument('--duration', type=float, default=5)
    ap.add_argument('--speed', type=int, default=60)
    ap.add_argument('--threshold', type=int, default=500)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--stop-only', action='store_true', help='Send zero-speed packets only; no movement')
    args = ap.parse_args()
    if args.stop_only:
        stop_robot(None, args.ip)
        return
    if not (0 < args.duration <= 30 and 0 < args.speed <= 100):
        ap.error('duration must be 0-30 seconds; speed 1-100')
    output = Path(__file__).resolve().parent / 'results'
    output.mkdir(exist_ok=True)
    path = output / ('search_safe_' + time.strftime('%Y%m%d_%H%M%S') + '.csv')
    rng = random.Random(args.seed)
    sock = None
    start = None
    last_left = last_right = 0
    print('SEARCH prototype (not full CPFA). Use elevated wheels for first test.')
    print('Power switch must remain accessible.')
    try:
        sock = socket.create_connection((args.ip, PORT), timeout=4)
        sock.settimeout(2)
        exchange(sock, 0, 0)
        start = time.monotonic()
        next_turn = start + rng.uniform(2, 4)
        turn_until = start
        turn_type = None
        direction = 1
        avoidance_events = 0
        with path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['seconds','state','left_speed','right_speed','max_proximity','proximity_0_to_7'])
            while time.monotonic() - start < args.duration:
                prox = exchange(sock, 0, 0)
                now = time.monotonic()
                if now - start >= args.duration:
                    break
                if max(prox) >= args.threshold and (turn_type != 'AVOID' or now >= turn_until):
                    avoidance_events += 1
                    direction = -1 if sum(prox[:4]) > sum(prox[4:]) else 1
                    turn_until = now + 0.55
                    turn_type = 'AVOID'
                elif now >= turn_until:
                    turn_type = None
                if turn_type is None and now >= next_turn:
                    direction = rng.choice((-1, 1))
                    turn_until = now + rng.uniform(0.25, 0.65)
                    next_turn = now + rng.uniform(2, 4)
                    turn_type = 'SEARCH_TURN'
                if turn_type:
                    state = turn_type
                    left, right = -args.speed * direction, args.speed * direction
                else:
                    state = 'SEARCH_FORWARD'
                    left = right = args.speed
                exchange(sock, left, right)
                last_left, last_right = left, right
                elapsed = time.monotonic() - start
                writer.writerow([round(elapsed, 3), state, left, right, max(prox), ' '.join(map(str, prox))])
                print(f'{elapsed:5.1f}s {state:15} max_prox={max(prox)}')
                time.sleep(0.08)
            writer.writerow([round(time.monotonic()-start, 3), 'STOP_REQUESTED', 0, 0, '', ''])
        print('Avoidance events:', avoidance_events)
        print('CSV saved:', path)
    except (KeyboardInterrupt, Exception) as exc:
        print('Stopping:', type(exc).__name__, str(exc))
    finally:
        stop_robot(sock, args.ip)

if __name__ == '__main__':
    main()
