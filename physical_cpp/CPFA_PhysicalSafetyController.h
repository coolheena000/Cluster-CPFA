#ifndef CPFA_PHYSICAL_SAFETY_CONTROLLER_H
#define CPFA_PHYSICAL_SAFETY_CONTROLLER_H

#include "CPFA_PhysicalNavigation.h"
#include "CPFA_PhysicalCollision.h"

class CPFAPhysicalSafetyController {
public:
    CPFAMovement decide(
        const CPFAPose& pose,
        const CPFATarget& target,
        const EpuckSensors& sensors,
        const std::array<double, 8>& sensorAngles,
        bool anglesVerified
    ) const {
        // Safety first: do not move with unverified inputs.
        if (!pose.valid || !sensors.valid || !anglesVerified)
            return CPFAMovement::STOP;

        CPFAPhysicalCollision collision;
        auto obstacle = collision.calculate(
            sensors, sensorAngles, anglesVerified
        );

        if (obstacle.obstacleDetected)
            return obstacle.movement;

        CPFAPhysicalNavigation navigation;
        return navigation.calculate(pose, target).movement;
    }
};

#endif