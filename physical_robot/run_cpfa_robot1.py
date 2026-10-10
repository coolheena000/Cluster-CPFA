"""Safe CPFA integration rehearsal. OFFLINE ONLY: no socket or motor commands."""
import argparse
import csv
from pathlib import Path
from cpfa_state_engine import CPFAEngine
from cpfa_navigation import Pose, waypoint_velocity, integrate_pose
from run_robot1_search_safe import packet, stop_robot  # import only; NEVER call stop_robot here


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dry-run', action='store_true', required=True)
    parser.add_argument('--steps', type=int, default=120)
    args = parser.parse_args()
    if not 1 <= args.steps <= 2000:
        parser.error('--steps must be 1..2000')
    engine = CPFAEngine(p_switch=1.0, p_return=0.0)
    pose = Pose(0.0, 0.0, 0.0)
    # This virtual food location is ONLY for the simulation.
    food = [(0.3, 0.0)]
    output = Path(__file__).resolve().parent / 'results' / 'cpfa_robot1_DRY_RUN.csv'
    output.parent.mkdir(exist_ok=True)
    states = set()
    with output.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['sim_time_s','state','x_sim_m','y_sim_m','heading_sim_rad','target_x','target_y','left_m_s_sim','right_m_s_sim','event'])
        for i in range(args.steps):
            state, target, event = engine.step((pose.x, pose.y), pose.heading, food)
            states.add(state)
            left, right, arrived = waypoint_velocity(pose, target)
            # Simulate motion: these velocities are NOT converted to hardware motor units.
            writer.writerow([i*0.5, state, pose.x, pose.y, pose.heading, *target, left, right, event])
            pose = integrate_pose(pose, left, right, 0.5)
    assert len(packet(0, 0)) == 21
    print('PASS: CPFA engine + navigation + existing e-puck Wi-Fi packet API loaded')
    print('States seen:', ', '.join(sorted(states)))
    print('Simulated food deliveries:', engine.food_collected)
    print('CSV:', output)
    print('DRY RUN ONLY: no Wi-Fi connection, motor movement, or physical localization.')

if __name__ == '__main__':
    main()
