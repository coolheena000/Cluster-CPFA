#include "CPFA_PhysicalController.h"
#include <cassert>
#include <iostream>

int main() {
    CPFAPhysicalController controller;

    CPFAPose pose{};
    pose.valid = false;

    EpuckSensors sensors{};
    sensors.valid = false;

    std::array<double, 8> sensorAngles = kEpuckSensorAngles;

    CPFAMovement movement = controller.decideMovement(
        pose,
        sensors,
        sensorAngles,
        false
    );

    assert(movement == CPFAMovement::STOP);

    std::cout << "INVALID POSE SAFETY TEST PASS\n";
    return 0;
}
