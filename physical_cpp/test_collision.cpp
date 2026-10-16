
#include "CPFA_PhysicalCollision.h"
#include "CPFA_EpuckSensorGeometry.h"
#include <cassert>
#include <iostream>

int main() {
    CPFAPhysicalCollision collision;
    EpuckSensors sensors{};
    sensors.valid = true;

    // IR0: front-right obstacle -> LEFT
    sensors.proximity[0] = 1000;
    auto result = collision.calculate(
        sensors, kEpuckSensorAngles, true
    );
    assert(result.obstacleDetected);
    assert(result.movement == CPFAMovement::LEFT);

    // IR7: front-left obstacle -> RIGHT
    sensors.proximity[0] = 0;
    sensors.proximity[7] = 1000;
    result = collision.calculate(
        sensors, kEpuckSensorAngles, true
    );
    assert(result.obstacleDetected);
    assert(result.movement == CPFAMovement::RIGHT);

    std::cout << "COLLISION LEFT/RIGHT TEST PASS\n";
    return 0;
}
