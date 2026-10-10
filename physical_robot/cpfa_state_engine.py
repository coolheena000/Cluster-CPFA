"""Offline CPFA state-transition prototype adapted from source/CPFA/CPFA_controller.cpp.
NOT a physical CPFA port. Coordinates/food detection are provided by a simulated observer.
No network, camera, or motor commands. Distances are arbitrary consistent units.
"""
from dataclasses import dataclass, field
import math
import random


def poisson_cdf(k, lam):
    return math.exp(-lam) * sum(lam ** i / math.factorial(i) for i in range(int(k) + 1))


@dataclass
class CPFAEngine:
    nest: tuple = (0.0, 0.0)
    nest_radius: float = 0.15
    target_tolerance: float = 0.1
    food_radius: float = 0.12
    step_size: float = 0.15
    p_switch: float = 0.12
    p_return: float = 0.08
    uninformed_variation: float = 0.4
    informed_decay: float = 0.02
    fidelity_rate: float = 1.0
    pheromone_rate: float = 1.0
    seed: int = 42
    state: str = 'DEPARTING'
    target: tuple = (0.8, 0.0)
    carrying: bool = False
    informed: bool = False
    fidelity: tuple | None = None
    food_collected: int = 0
    search_steps: int = 0
    resource_density: int = 0
    survey_step: int = 0
    trail: list = field(default_factory=list)
    events: list = field(default_factory=list)
    def __post_init__(self):
        self.rng = random.Random(self.seed)

    def _near(self, a, b, r):
        return math.dist(a, b) <= r

    def _new_search_target(self, pos, heading):
        variation = self.uninformed_variation
        if self.informed:
            variation += (2 * math.pi - variation) * math.exp(-self.informed_decay * self.search_steps)
        theta = heading + self.rng.gauss(0, variation)
        self.target = (pos[0] + self.step_size * math.cos(theta),
                       pos[1] + self.step_size * math.sin(theta))
        self.search_steps += 1

    def step(self, position, heading, food_positions):
        """Advance one logical 0.5-second decision tick; caller supplies position and visible food.
        Returns state, target, event. Never commands real motors.
        """
        event = ''
        if self.state == 'DEPARTING':
            if self.informed and self._near(position, self.target, self.target_tolerance):
                self.state = 'SEARCHING'
                self.search_steps = 0
                event = 'informed_target_reached'
            elif not self.informed and self.rng.random() < self.p_switch:
                self.state = 'SEARCHING'
                self.search_steps = 0
                self._new_search_target(position, heading)
                event = 'begin_uninformed_search'
            elif self._near(position, self.target, self.target_tolerance):
                self.target = (self.rng.uniform(-1, 1), self.rng.uniform(-1, 1))
                event = 'new_departure_target'
        elif self.state == 'SEARCHING':
            found = next((f for f in food_positions if self._near(position, f, self.food_radius)), None)
            if found is not None:
                self.carrying = True
                self.fidelity = tuple(found)
                self.resource_density = 1 + sum(self._near(found, f, self.food_radius * 2) for f in food_positions if f != found)
                self.trail = [tuple(position)]
                self.survey_step = 0
                self.state = 'SURVEYING'
                event = 'food_detected_SIMULATED'
            elif self._near(position, self.target, self.target_tolerance):
                if self.rng.random() < self.p_return:
                    self.state = 'RETURNING'
                    self.target = self.nest
                    self.informed = False
                    event = 'gave_up_search'
                else:
                    self._new_search_target(position, heading)
                    event = 'next_search_step'
        elif self.state == 'SURVEYING':
            # Original controller increments survey_count on heading tolerance.
            desired = (self.survey_step * math.pi / 2) % (2 * math.pi)
            err = (heading - desired + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < 0.12:
                self.survey_step += 1
            if self.survey_step > 4:
                self.state = 'RETURNING'
                self.target = self.nest
                event = 'survey_complete'
            else:
                self.target = (position[0] + 0.05 * math.cos(desired), position[1] + 0.05 * math.sin(desired))
        elif self.state == 'RETURNING':
            if self.carrying:
                self.trail.append(tuple(position))
            if self._near(position, self.nest, self.nest_radius):
                if self.carrying:
                    self.food_collected += 1
                    event = 'food_delivered_SIMULATED'
                p_fidelity = poisson_cdf(self.resource_density, self.fidelity_rate)
                if self.fidelity is not None and self.rng.random() < p_fidelity:
                    self.target = self.fidelity
                    self.informed = True
                    event += '|site_fidelity'
                else:
                    # Pheromone following requires shared trails; single robot offline fallback.
                    self.target = (self.rng.uniform(-1, 1), self.rng.uniform(-1, 1))
                    self.informed = False
                    event += '|random_departure'
                self.carrying = False
                self.state = 'DEPARTING'
                self.trail.clear()
        self.events.append((self.state, event))
        return self.state, self.target, event
