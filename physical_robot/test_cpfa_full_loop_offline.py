"""Combined OFFLINE CPFA decision + differential drive navigation smoke test.
No socket, camera, motor, or real food detection. Does not overwrite any files.
"""
import csv
import math
from pathlib import Path
from cpfa_state_engine import CPFAEngine
from cpfa_navigation import Pose, NavConfig, waypoint_velocity, integrate_pose, wrap_angle


def main():
    cfg = NavConfig()
    cpfa = CPFAEngine(seed=4, p_switch=1.0, p_return=0.0, target=(0.5, 0.0),
                      food_radius=0.09, nest_radius=0.12)
    pose = Pose(0.0, 0.0, 0.0)
    virtual_food = [(0.20, 0.0)]
    dt = 0.1
    rows = []
    visited = set()
    events = []
    for tick in range(2400):
        t = tick * dt
        # CPFA decisions every 0.5 s, matching the original logical tick cadence.
        if tick % 5 == 0:
            state, target, event = cpfa.step((pose.x, pose.y), pose.heading, virtual_food)
            if event:
                events.append((round(t, 1), event))
                if event == 'food_detected_SIMULATED':
                    virtual_food.clear()  # simulate pickup only once
            visited.add(state)
        state = cpfa.state
        if state == 'SURVEYING':
            # Survey is an orientation task, not ordinary waypoint navigation.
            desired = (cpfa.survey_step * math.pi / 2) % (2 * math.pi)
            err = wrap_angle(desired - pose.heading)
            omega = max(-0.8, min(0.8, 2.0 * err))
            left, right = -omega * cfg.wheel_base_m / 2, omega * cfg.wheel_base_m / 2
        else:
            left, right, _ = waypoint_velocity(pose, cpfa.target, cfg)
        pose = integrate_pose(pose, left, right, dt, cfg)
        rows.append([f'{t:.1f}', state, f'{pose.x:.4f}', f'{pose.y:.4f}',
                     f'{pose.heading:.4f}', f'{cpfa.target[0]:.4f}',
                     f'{cpfa.target[1]:.4f}', f'{left:.5f}', f'{right:.5f}',
                     cpfa.food_collected])
        if cpfa.food_collected >= 1:
            break
    out = Path(__file__).resolve().parent / 'results' / 'cpfa_full_loop_OFFLINE.csv'
    out.parent.mkdir(exist_ok=True)
    with out.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'state', 'sim_x_m', 'sim_y_m', 'sim_heading_rad',
                         'target_x_m', 'target_y_m', 'left_m_s', 'right_m_s', 'virtual_food_delivered'])
        writer.writerows(rows)
    print('States visited:', ' -> '.join(dict.fromkeys(row[1] for row in rows)))
    print('Events:', events)
    print('Virtual food delivered:', cpfa.food_collected)
    print('CSV:', out)
    assert cpfa.food_collected == 1, 'Offline run did not complete virtual delivery'
    assert {'SEARCHING', 'SURVEYING', 'RETURNING'}.issubset(visited)
    print('PASS: CPFA engine + waypoint navigation + simulated motion integrated')
    print('NOTE: OFFLINE ONLY. No robot, camera, or physical food collection.')

if __name__ == '__main__':
    main()
