
#ifndef CPFA_PHYSICAL_INTEGRATION_H
#define CPFA_PHYSICAL_INTEGRATION_H

#include "CPFA_PhysicalBridge.h"
#include "CPFA_PhysicalController.h"

class CPFAPhysicalIntegration {
public:
    CPFAPhysicalIntegration(
        CPFAPhysicalBridge& bridge,
        CPFAPhysicalController& controller
    ) : bridge_(bridge), controller_(controller) {}

    CPFAMovement decide(bool anglesVerified) {
        return controller_.decideMovement(
            bridge_.getPose(),
            bridge_.getSensors(),
            kEpuckSensorAngles,
            anglesVerified
        );
    }

    bool executeStop() {
        return bridge_.move(CPFAMovement::STOP, 0);
    }

    void stepCPFA(
        bool halfSecondTick,
        bool foodDetected,
        bool inNest,
        double randomNumber,
        double gaussianSample,
        double randomX,
        double randomY,
        double randomLay,
        double randomFidelity,
        double randomPheromone,
        CPFATarget randomSearchTarget,
        double timeSeconds
    ) {
        controller_.step(
            bridge_.getPose(),
            halfSecondTick,
            foodDetected,
            inNest,
            randomNumber,
            gaussianSample,
            randomX,
            randomY,
            randomLay,
            randomFidelity,
            randomPheromone,
            randomSearchTarget,
            timeSeconds
        );
    }

private:
    CPFAPhysicalBridge& bridge_;
    CPFAPhysicalController& controller_;
};

#endif
