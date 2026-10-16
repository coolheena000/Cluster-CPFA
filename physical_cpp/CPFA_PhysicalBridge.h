#ifndef CPFA_PHYSICAL_BRIDGE_H
#define CPFA_PHYSICAL_BRIDGE_H

#include "EpuckHardware.h"
#include "CPFA_MotorAdapter.h"
#include "CPFA_Localization.h"
#include <algorithm>
#include <cstdint>

class CPFAPhysicalBridge {
public:
    explicit CPFAPhysicalBridge(EpuckHardware& robot)
        : robot_(robot), motors_(robot) {}

    bool updateSensors() {
        EpuckSensors data{};
        if (!robot_.readSensors(data) || !data.valid) {
            localization_.invalidate();
            return false;
        }
        sensors_ = data;
        return true;
    }

    bool sendRequest() {
        return robot_.sendRequest();
    }

    bool receiveSensors() {
        EpuckSensors data{};
        if (!robot_.receiveSensors(data) || !data.valid) {
            localization_.invalidate();
            return false;
        }
        sensors_ = data;
        return true;
    }

    void invalidatePose() {
        localization_.invalidate();
    }

    void updatePose(double x, double y, double heading) {
        localization_.update(x, y, heading);
    }

    CPFAPose getPose() const {
        return localization_.getPose();
    }
    EpuckSensors getSensors() const {
    return sensors_;
}

    uint16_t maxProximity() const {
        return *std::max_element(
            sensors_.proximity.begin(),
            sensors_.proximity.end());
    }

    bool move(CPFAMovement state, int16_t speed) {
        if (!localization_.getPose().valid &&
            state != CPFAMovement::STOP) {
            motors_.move(CPFAMovement::STOP, 0);
            return false;
        }
        return motors_.move(state, speed);
    }

private:
    EpuckHardware& robot_;
    CPFAMotorAdapter motors_;
    CPFALocalization localization_;
    EpuckSensors sensors_{};
};

#endif
