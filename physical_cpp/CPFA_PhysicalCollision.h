#ifndef CPFA_PHYSICAL_COLLISION_H
#define CPFA_PHYSICAL_COLLISION_H

#include "CPFA_MotorAdapter.h"
#include "EpuckHardware.h"

#include <array>
#include <cmath>

class CPFAPhysicalCollision {
public:
    struct Decision {
        bool obstacleDetected = false;
        CPFAMovement movement = CPFAMovement::STOP;
        double collisionAngle = 0.0;
    };

    Decision calculate(
        const EpuckSensors& sensors,
        const std::array<double, 8>& sensorAngles,
        bool anglesVerified
    ) const {
        Decision result{};

        // Never command movement without valid sensor information.
        if (!sensors.valid || !anglesVerified)
            return result;

        double x = 0.0;
        double y = 0.0;

        // Carlos's original GetCollisionVector() approach.
        for (size_t i = 0; i < 8; ++i) {
            x += sensors.proximity[i] * std::cos(sensorAngles[i]);
            y += sensors.proximity[i] * std::sin(sensorAngles[i]);
        }

        x /= 8.0;
        y /= 8.0;

        if (std::hypot(x, y) < 20.0) // ignore IR noise
            return result;

        result.collisionAngle =
            std::atan2(y, x) * 180.0 / 3.141592653589793;

        // Carlos's original front collision range.
        if (result.collisionAngle >= -37.5 &&
            result.collisionAngle <= 37.5) {

            result.obstacleDetected = true;

            // Preserve Carlos's turn-away direction.
            result.movement =
                result.collisionAngle <= 0.0
                    ? CPFAMovement::LEFT
                    : CPFAMovement::RIGHT;
        }

        return result;
    }
};

#endif