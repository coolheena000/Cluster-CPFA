"""Offline integration smoke test: CPFA state engine + physical Wi-Fi controller API.
No robot connection or motor movement. Does NOT validate real localization/foraging.
Run from project root: python3 physical_robot/test_cpfa_integration.py
"""
import csv
import math
from pathlib import Path
from cpfa_state_engine import CPFAEngine
from run_robot1_search_safe import packet, exchange, stop_robot


def run():
    assert len(packet(0, 0)) == 21
    assert packet(0, 0) == bytes([0x80, 2, 0] + [0] * 18)
    assert callable(exchange) and callable(stop_robot)
    print('PASS: physical Wi-Fi motor packet and STOP interface imported (not transmitted)')

    engine = CPFAEngine(seed=42, p_switch=1.0, p_return=0.0)
    # Artificial position updates; these are NOT robot localization readings.
    pos = (0.0, 0.0)
    heading = 0.0
    records = []
    food = [(0.45, 0.0)]
    for tick in range(50):
        state, target, event = engine.step(pos, heading, food)
        records.append([tick, state, round(pos[0], 3), round(pos[1], 3), round(heading, 3), event])
        if state == 'SURVEYING':
            # Simulate a camera reporting the target heading.
            heading = (engine.survey_step * math.pi / 2) % (2 * math.pi)
        elif state == 'RETURNING':
            pos = engine.nest
        else:
            # Simulate a camera reporting arrival at the next waypoint.
            pos = target
            heading = 0.0
        if engine.food_collected:
            break
    out = Path(__file__).resolve().parent / 'results'
    out.mkdir(exist_ok=True)
    csv_path = out / 'cpfa_integration_OFFLINE.csv'
    with csv_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['tick','state','sim_x','sim_y','sim_heading','event'])
        writer.writerows(records)
    print('States visited:', ' -> '.join(dict.fromkeys(r[1] for r in records)))
    print('Simulated food delivered:', engine.food_collected)
    print('Offline CSV:', csv_path)
    print('NOTE: NO physical robot, camera, or food sensing was used.')
    assert engine.food_collected == 1, 'Offline path did not complete simulated delivery'
    print('PASS: state engine and hardware interface are compatible at import/API level')


if __name__ == '__main__':
    run()
