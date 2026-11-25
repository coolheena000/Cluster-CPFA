// Offline test: Carlos's collision maneuver (turn 37.5 -/+ angle, then forward SearchStepSize).
#include "CPFA_CollisionManeuver.h"
#include "CPFA_EpuckSensorGeometry.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

int main() {
    CPFACollisionManeuver m(200, 150);
    CPFAPhysicalCollision collision;
    const double dt = 0.125;  // 8 Hz

    // Obstacle front-right (IR0) -> collision angle < 0 -> LEFT turn.
    EpuckSensors s{};
    s.valid = true;
    s.proximity[0] = 1000;
    auto d = collision.calculate(s, kEpuckSensorAngles, true);
    assert(d.obstacleDetected && d.collisionAngle < 0.0);

    CPFAMovement mv;
    int16_t sp;
    assert(m.update(d, dt, mv, sp) && mv == CPFAMovement::LEFT && sp == 150);

    // Obstacle gone: the maneuver keeps turning, then drives forward, then ends.
    s.proximity[0] = 0;
    d = collision.calculate(s, kEpuckSensorAngles, true);
    int turnTicks = 1, forwardTicks = 0;
    while (m.update(d, dt, mv, sp)) {
        if (mv == CPFAMovement::LEFT) { assert(forwardTicks == 0); ++turnTicks; }
        else { assert(mv == CPFAMovement::FORWARD && sp == 200); ++forwardTicks; }
    }

    const double expectedTurnDeg = 37.5 + 17.0;  // IR0 is at -17 deg
    const double turnedDeg = turnTicks * dt * (2.0 * 150 * CPFACollisionManeuver::kMetresPerStep /
                                               CPFACollisionManeuver::kAxle) * 180.0 / 3.141592653589793;
    const double forwardM = forwardTicks * dt * 200 * CPFACollisionManeuver::kMetresPerStep;
    std::printf("turn ~%.0f deg (Carlos: %.0f), forward %.3f m (Carlos: 0.080)\n",
                turnedDeg, expectedTurnDeg, forwardM);
    assert(std::fabs(turnedDeg - expectedTurnDeg) < 10.0);
    assert(std::fabs(forwardM - 0.08) < 0.01);
    assert(!m.active() && m.count() == 1);

    // Obstacle front-left (IR7) -> RIGHT turn.
    s.proximity[7] = 1000;
    d = collision.calculate(s, kEpuckSensorAngles, true);
    assert(m.update(d, dt, mv, sp) && mv == CPFAMovement::RIGHT);

    std::cout << "COLLISION MANEUVER TEST PASS\n";
    return 0;
}
