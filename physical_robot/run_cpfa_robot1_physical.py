"""Bounded physical SEARCHING test for e-puck2; NOT complete Carlos CPFA.
Reuses the independently tested Wi-Fi protocol and fresh-connection STOP.
"""
import argparse
import csv
import random
import socket
import time
from pathlib import Path
from run_robot1_search_safe import exchange, stop_robot, PORT


def main():
    ap = argparse.ArgumentParser(description='Physical CPFA-inspired SEARCHING stage only')
    ap.add_argument('--ip', default='172.20.10.2')
    ap.add_argument('--duration', type=float, default=3)
    ap.add_argument('--speed', type=int, default=30)
    ap.add_argument('--threshold', type=int, default=500)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--enable-motors', action='store_true', help='Explicitly permit motor movement')
    ap.add_argument('--stop-only', action='store_true')
    args = ap.parse_args()
    if args.stop_only:
        stop_robot(None, args.ip)
        return
    if not (0 < args.duration <= 5 and 0 < args.speed <= 60 and 1 <= args.threshold <= 65535):
        ap.error('duration must be >0 and <=5 s, speed 1..60, threshold 1..65535')
    if not args.enable_motors:
        print('DRY RUN: no Wi-Fi connection or motor command sent.')
        print('Would run bounded SEARCHING stage for', args.duration, 'seconds at speed', args.speed)
        print('This is NOT complete CPFA: no real localization or food detection.')
        return
    print('PHYSICAL SEARCHING TEST. Lift wheels off ground; keep power switch accessible.')
    print('Press Ctrl+C to request STOP; switch power OFF if wheels do not stop.')
    rng = random.Random(args.seed)
    out = Path(__file__).resolve().parent / 'results'
    out.mkdir(exist_ok=True)
    csv_path = out / ('cpfa_search_PHYSICAL_' + time.strftime('%Y%m%d_%H%M%S') + '.csv')
    sock = None
    try:
        sock = socket.create_connection((args.ip, PORT), timeout=4)
        sock.settimeout(1.5)
        exchange(sock, 0, 0)
        start = time.monotonic()
        next_turn = start + rng.uniform(1.0, 1.8)
        turn_until = start
        mode = 'FORWARD'
        direction = 1
        with csv_path.open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['elapsed_s','state','left','right','max_proximity','proximity_0_to_7'])
            while time.monotonic() - start < args.duration:
                proximity = exchange(sock, 0, 0)
                now = time.monotonic()
                if now - start >= args.duration:
                    break
                if max(proximity) >= args.threshold:
                    direction = -1 if sum(proximity[:4]) > sum(proximity[4:]) else 1
                    mode = 'AVOID'
                    turn_until = now + 0.4
                elif now >= turn_until:
                    mode = 'FORWARD'
                if mode == 'FORWARD' and now >= next_turn:
                    direction = rng.choice((-1, 1))
                    mode = 'SEARCH_TURN'
                    turn_until = now + rng.uniform(0.2, 0.35)
                    next_turn = now + rng.uniform(1.0, 1.8)
                if mode == 'FORWARD':
                    left = right = args.speed
                else:
                    left, right = -args.speed * direction, args.speed * direction
                exchange(sock, left, right)
                elapsed = time.monotonic() - start
                w.writerow([round(elapsed, 3), 'SEARCHING_'+mode, left, right, max(proximity), ' '.join(map(str, proximity))])
                print(f'{elapsed:.2f}s SEARCHING_{mode} max_prox={max(proximity)}')
                time.sleep(0.08)
        print('CSV:', csv_path)
    except (KeyboardInterrupt, Exception) as exc:
        print('STOP triggered:', type(exc).__name__, str(exc))
    finally:
        stop_robot(sock, args.ip)


if __name__ == '__main__':
    main()
