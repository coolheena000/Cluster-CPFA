"""Single e-puck2 CPFA-inspired uninformed search demonstration.
Not a complete CPFA port: no physical food/nest localization, pheromone or site fidelity.
"""
import argparse
import csv
import random
import socket
import struct
import time
from pathlib import Path


def packet(left=0, right=0):
    data = bytearray(21)
    data[0:3] = b'\x80\x02\x00'
    struct.pack_into('<hh', data, 3, left, right)
    return data


def exact(sock, n):
    out = bytearray()
    while len(out) < n:
        part = sock.recv(n - len(out))
        if not part:
            raise ConnectionError('Robot disconnected')
        out.extend(part)
    return out


def exchange(sock, left, right):
    sock.sendall(packet(left, right))
    if exact(sock, 1) != b'\x02':
        raise RuntimeError('Unexpected sensor header')
    raw = exact(sock, 104)
    return [raw[37 + 2*i] + 256*raw[38 + 2*i] for i in range(8)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='172.20.10.2')
    ap.add_argument('--duration', type=float, default=30)
    ap.add_argument('--speed', type=int, default=100)
    ap.add_argument('--threshold', type=int, default=500)
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()
    if not (0 < args.duration <= 120 and 0 < args.speed <= 150):
        ap.error('duration must be 0-120 seconds; speed 1-150')
    rng = random.Random(args.seed)
    output = Path(__file__).resolve().parent / 'results'
    output.mkdir(exist_ok=True)
    path = output / ('search_' + time.strftime('%Y%m%d_%H%M%S') + '.csv')
    print('CPFA-inspired SEARCH prototype; not full physical CPFA.')
    print('Keep robot on floor in a clear, bounded test area. Power switch is emergency stop.')
    print('Press Ctrl+C to request stop.')
    sock = None
    try:
        sock = socket.create_connection((args.ip, 1000), timeout=5)
        sock.settimeout(2)
        exchange(sock, 0, 0)
        start = time.monotonic()
        turn_until = start
        next_turn = start + rng.uniform(2, 4)
        direction = 1
        avoid_events = 0
        with path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['seconds','state','left_speed','right_speed','max_proximity','proximity_0_to_7'])
            while time.monotonic() - start < args.duration:
                now = time.monotonic()
                # Previous sensor sample, and periodic randomized heading changes.
                prox = exchange(sock, 0, 0)
                obstacle = max(prox) >= args.threshold
                if obstacle and now >= turn_until:
                    avoid_events += 1
                    direction = -1 if sum(prox[:4]) > sum(prox[4:]) else 1
                    turn_until = now + 0.55
                if now < turn_until:
                    state = 'AVOID'
                    left, right = (-args.speed * direction, args.speed * direction)
                elif now >= next_turn:
                    direction = rng.choice([-1, 1])
                    turn_until = now + rng.uniform(0.25, 0.65)
                    next_turn = now + rng.uniform(2, 4)
                    state = 'SEARCH_TURN'
                    left, right = (-args.speed * direction, args.speed * direction)
                else:
                    state = 'SEARCH_FORWARD'
                    left = right = args.speed
                exchange(sock, left, right)
                writer.writerow([round(time.monotonic()-start, 3),state,left,right,max(prox),' '.join(map(str,prox))])
                print(f'{time.monotonic()-start:5.1f}s {state:15} max_prox={max(prox)}')
                time.sleep(0.08)
        print('Avoidance events:', avoid_events)
        print('CSV saved:', path)
    except (KeyboardInterrupt, Exception) as exc:
        print('Stopping:', type(exc).__name__, str(exc))
    finally:
        if sock:
            try:
                sock.settimeout(2)
                exchange(sock, 0, 0)
                print('STOP command acknowledged (verify wheels stopped).')
            except Exception as error:
                print('STOP not confirmed:', error)
                print('Switch robot OFF if wheels are moving.')
            finally:
                sock.close()

if __name__ == '__main__':
    main()
