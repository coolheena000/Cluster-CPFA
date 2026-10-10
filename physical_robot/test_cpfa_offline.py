"""No robot connection required: deterministic state transition checks."""
from cpfa_state_engine import CPFAEngine, poisson_cdf

def run():
    assert abs(poisson_cdf(1, 1) - 2 / 2.718281828459045) < 1e-9
    cpfa = CPFAEngine(p_switch=1.0, p_return=0.0)
    state, _, event = cpfa.step((0, 0), 0, [])
    assert state == 'SEARCHING', (state, event)
    state, _, event = cpfa.step((0.5, 0.2), 0, [(0.5, 0.2)])
    assert state == 'SURVEYING' and cpfa.carrying
    for angle in [0, math.pi/2, math.pi, 3*math.pi/2, 0]:
        state, _, event = cpfa.step((0.5, 0.2), angle, [])
    assert state == 'RETURNING', (state, event)
    state, _, event = cpfa.step((0, 0), 0, [])
    assert cpfa.food_collected == 1 and state == 'DEPARTING'
    print('PASS: DEPARTING -> SEARCHING -> SURVEYING -> RETURNING -> DEPARTING')
    print('PASS: simulated food delivered, count =', cpfa.food_collected)
    print('NOTE: offline state-machine tests, NOT physical food collection')

if __name__ == '__main__':
    import math
    run()
