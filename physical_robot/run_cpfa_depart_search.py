"""Bounded e-puck2 DEPARTING -> SEARCHING behavior demo.
NOT a faithful physical port of Carlos CPFA: state change is time-triggered,
not position/probability-triggered. No physical food or nest detection.
Uses previously physically verified Wi-Fi and fresh-connection STOP routines.
"""
import argparse
import csv
import random
import socket
import time
from pathlib import Path
from run_robot1_search_safe import exchange, stop_robot, PORT


def commands(elapsed, depart_seconds, speed, proximity, threshold, rng, turn_state):
    """Return state, wheel speeds, and updated search turn schedule."""
    if max(proximity) >= threshold:
        direction = -1 if sum(proximity[:4]) > sum(proximity[4:]) else 1
        return 'OBSTACLE_AVOID', -speed * direction, speed * direction, turn_state
    if elapsed < depart_seconds:
        return 'DEPARTING_FORWARD', speed, speed, turn_state
    turn_until, next_turn, direction = turn_state
    if elapsed >= next_turn and elapsed >= turn_until:
        direction = rng.choice((-1, 1))
        turn_until = elapsed + rng.uniform(0.2, 0.35)
        next_turn = elapsed + rng.uniform(0.8, 1.3)
    if elapsed < turn_until:
        return 'SEARCHING_TURN', -speed * direction, speed * direction, (turn_until, next_turn, direction)
    return 'SEARCHING_FORWARD', speed, speed, (turn_until, next_turn, direction)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ip', default='172.20.10.2')
    p.add_argument('--duration', type=float, default=3.0)
    p.add_argument('--depart-seconds', type=float, default=1.0)
    p.add_argument('--speed', type=int, default=30)
    p.add_argument('--threshold', type=int, default=500)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--enable-motors', action='store_true')
    p.add_argument('--stop-only', action='store_true')
    a = p.parse_args()
    if a.stop_only:
        stop_robot(None, a.ip)
        return
    if not (0 < a.duration <= 5 and 0 < a.depart_seconds < a.duration and 1 <= a.speed <= 60 and 1 <= a.threshold <= 65535):
        p.error('Require 0 < depart-seconds < duration <= 5, speed 1..60, threshold 1..65535')
    rng = random.Random(a.seed)
    print('Timed DEPARTING -> SEARCHING prototype; NOT full Carlos CPFA.')
    if not a.enable_motors:
        state = (0.0, a.depart_seconds + 0.8, 1)
        for i in range(int(a.duration / 0.1)):
            t = i * 0.1
            mode, left, right, state = commands(t, a.depart_seconds, a.speed, [0]*8, a.threshold, rng, state)
            if i == 0 or (i > 0 and (t - 0.1 < a.depart_seconds <= t)) or mode == 'SEARCHING_TURN':
                print(f'{t:.1f}s {mode} left={left} right={right}')
        print('PASS: DRY RUN ONLY; no socket opened or motor command sent.')
        return
    print('LIFT WHEELS OFF FLOOR. Keep physical power switch accessible.')
    print('If wheels do not stop, immediately switch robot OFF.')
    out = Path(__file__).resolve().parent / 'results'
    out.mkdir(exist_ok=True)
    log_path = out / ('cpfa_depart_search_PHYSICAL_' + time.strftime('%Y%m%d_%H%M%S') + '.csv')
    sock = None
    try:
        sock = socket.create_connection((a.ip, PORT), timeout=4)
        sock.settimeout(1.5)
        exchange(sock, 0, 0)
        start = time.monotonic()
        turn_state = (0.0, a.depart_seconds + 0.8, 1)
        with log_path.open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['elapsed_s','mode','left','right','max_proximity','proximity_0_to_7'])
            while True:
                prox = exchange(sock, 0, 0)
                elapsed = time.monotonic() - start
                if elapsed >= a.duration:
                    break
                mode, left, right, turn_state = commands(elapsed, a.depart_seconds, a.speed, prox, a.threshold, rng, turn_state)
                exchange(sock, left, right)
                w.writerow([round(elapsed,3), mode, left, right, max(prox), ' '.join(map(str,prox))])
                print(f'{elapsed:.2f}s {mode} max_prox={max(prox)}')
                time.sleep(0.08)
        print('CSV:', log_path)
    except (KeyboardInterrupt, Exception) as exc:
        print('STOP triggered:', type(exc).__name__, str(exc))
    finally:
        stop_robot(sock, a.ip)


if __name__ == '__main__':
    main()
