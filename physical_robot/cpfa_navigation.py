"""Offline differential-drive waypoint navigation for e-puck2.
No hardware I/O. Pose must come from a real localization source in physical use.
"""
from dataclasses import dataclass
from math import atan2, cos, sin, hypot, pi


def wrap_angle(angle):
    return (angle + pi) % (2*pi) - pi


@dataclass
class Pose:
    x: float
    y: float
    heading: float  # radians


@dataclass
class NavConfig:
    wheel_radius_m: float = 0.0205  # approximate; calibrate on your robot
    wheel_base_m: float = 0.053    # approximate; calibrate on your robot
    max_linear_m_s: float = 0.06
    max_angular_rad_s: float = 1.5
    heading_gain: float = 2.0
    arrival_radius_m: float = 0.05
    turn_in_place_rad: float = 0.45


def waypoint_velocity(pose, target, cfg=None):
    """Return (left_m_s, right_m_s, arrived). No robot commands are sent."""
    cfg = cfg or NavConfig()
    distance = hypot(target[0]-pose.x, target[1]-pose.y)
    if distance <= cfg.arrival_radius_m:
        return 0.0, 0.0, True
    error = wrap_angle(atan2(target[1]-pose.y, target[0]-pose.x)-pose.heading)
    omega = max(-cfg.max_angular_rad_s, min(cfg.max_angular_rad_s, cfg.heading_gain*error))
    v = 0.0 if abs(error) > cfg.turn_in_place_rad else min(cfg.max_linear_m_s, distance*0.8)
    left = v - omega*cfg.wheel_base_m/2
    right = v + omega*cfg.wheel_base_m/2
    return left, right, False


def integrate_pose(pose, left, right, dt, cfg=None):
    """Simulated motion ONLY; not a real-world position measurement."""
    cfg = cfg or NavConfig()
    v = (left+right)/2
    omega = (right-left)/cfg.wheel_base_m
    mid = pose.heading+omega*dt/2
    return Pose(pose.x+v*cos(mid)*dt, pose.y+v*sin(mid)*dt, wrap_angle(pose.heading+omega*dt))
