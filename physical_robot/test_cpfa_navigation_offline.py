"""Run: python3 physical_robot/test_cpfa_navigation_offline.py"""
import csv
from pathlib import Path
from cpfa_navigation import Pose, NavConfig, waypoint_velocity, integrate_pose

cfg = NavConfig()
pose = Pose(0, 0, 1.57)
targets = [(0.6, 0.0), (0.6, 0.5), (0.0, 0.0)]
rows = []
dt = 0.05
for index, target in enumerate(targets, 1):
    for tick in range(2500):
        left, right, arrived = waypoint_velocity(pose, target, cfg)
        rows.append([index, round(tick*dt, 3), pose.x, pose.y, pose.heading, left, right, arrived])
        if arrived:
            print(f'PASS: waypoint {index} reached at simulated ({pose.x:.3f}, {pose.y:.3f})')
            break
        pose = integrate_pose(pose, left, right, dt, cfg)
    else:
        raise RuntimeError(f'FAIL: waypoint {index} not reached')
path = Path(__file__).resolve().parent/'results'/'cpfa_navigation_OFFLINE.csv'
path.parent.mkdir(exist_ok=True)
with path.open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['waypoint','sim_time_s','x_m','y_m','heading_rad','left_m_s','right_m_s','arrived'])
    writer.writerows(rows)
print('PASS: offline waypoint navigation simulation')
print('CSV:', path)
print('NOTE: simulated pose only. NO Wi-Fi, motors, or real localization used.')
