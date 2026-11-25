#ifndef CPFA_MOTOR_ADAPTER_H
#define CPFA_MOTOR_ADAPTER_H

#include "EpuckHardware.h"
#include <cstdint>

enum class CPFAMovement {
    STOP, LEFT, RIGHT, FORWARD, BACK
};

class CPFAMotorAdapter {
public:
    explicit CPFAMotorAdapter(EpuckHardware& robot)
        : robot_(robot) {}

    bool move(CPFAMovement state, int16_t speed) {
        switch (state) {
            case CPFAMovement::STOP:
                return robot_.setWheelSpeeds(0, 0);
            case CPFAMovement::LEFT:
                return robot_.setWheelSpeeds(-speed, speed);
            case CPFAMovement::RIGHT:
                return robot_.setWheelSpeeds(speed, -speed);
            case CPFAMovement::FORWARD:
                return robot_.setWheelSpeeds(speed, speed);
            case CPFAMovement::BACK:
                return robot_.setWheelSpeeds(-speed, -speed);
        }
        return false;
    }

private:
    EpuckHardware& robot_;
};

#endif
