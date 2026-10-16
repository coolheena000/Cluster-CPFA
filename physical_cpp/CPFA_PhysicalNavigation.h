
#ifndef CPFA_PHYSICAL_NAVIGATION_H
#define CPFA_PHYSICAL_NAVIGATION_H

#include "CPFA_Localization.h"
#include "CPFA_MotorAdapter.h"
#include <cmath>

struct CPFATarget {
    double x = 0.0;
    double y = 0.0;
};

class CPFAPhysicalNavigation {
public:
    struct Decision {
        CPFAMovement movement = CPFAMovement::STOP;
        double distance = 0.0;
        double headingError = 0.0;
        bool atTarget = false;
    };

    Decision calculate(
        const CPFAPose& pose,
        const CPFATarget& target,
        bool headingToNest = false
    ) const {
        Decision result{};

        if (!pose.valid)
            return result;

        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;

        result.distance = std::hypot(dx, dy);

        const double targetDistanceTolerance = 0.05;
const double nestDistanceTolerance = 0.05;

const double distanceTolerance =
    headingToNest ? nestDistanceTolerance : targetDistanceTolerance;

const double angleTolerance =
    headingToNest ? 0.1 : 0.1;
        if (result.distance < distanceTolerance) {
            result.atTarget = true;
            return result;
        }

        const double targetHeading = std::atan2(dy, dx);

        result.headingError = std::atan2(
            std::sin(targetHeading - pose.heading),
            std::cos(targetHeading - pose.heading)
        );

        if (result.headingError > angleTolerance)
            result.movement = CPFAMovement::LEFT;
        else if (result.headingError < -angleTolerance)
            result.movement = CPFAMovement::RIGHT;
        else
            result.movement = CPFAMovement::FORWARD;

        return result;
    }
};

#endif
