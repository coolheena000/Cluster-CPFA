#ifndef CPFA_PHYSICAL_CONTROLLER_H
#define CPFA_PHYSICAL_CONTROLLER_H

#include "CPFA_PhysicalSafetyController.h"
#include "CPFA_EpuckSensorGeometry.h"
#include "CPFA_PhysicalCore.h"
#include <vector>

class CPFAPhysicalController {
public:
    CPFAPhysicalCore core;
            CPFAPhysicalSafetyController safetyController;

    CPFAMovement decideMovement(
        const CPFAPose& pose,
        const EpuckSensors& sensors,
        const std::array<double, 8>& sensorAngles,
        bool anglesVerified
    ) const {
        return safetyController.decide(
            pose,
            core.target,
            sensors,
            sensorAngles,
            anglesVerified
        );
    }

    std::vector<CPFAPhysicalPheromone>* sharedPheromones = nullptr;

    // One call represents one control tick.
    // At 16 ticks/second, halfSecondTick is true every 8 ticks.
    void step(
        const CPFAPose& pose,
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
        if (!pose.valid) return;

        switch (core.state) {
            case CPFAPhysicalCore::State::DEPARTING:
                core.departing(
                    pose,
                    randomNumber,
                    gaussianSample,
                    halfSecondTick,
                    randomX,
                    randomY
                );
                break;

            case CPFAPhysicalCore::State::SEARCHING:
                if (halfSecondTick) {
                    core.searching(
                        pose,
                        randomNumber,
                        gaussianSample,
                        foodDetected
                    );
                }
                break;

            case CPFAPhysicalCore::State::RETURNING:
                if (sharedPheromones != nullptr) {
                    core.returning(
                        pose,
                        inNest,
                        randomLay,
                        randomFidelity,
                        randomPheromone,
                        randomSearchTarget,
                        timeSeconds,
                        *sharedPheromones
                    );
                }
                break;

            case CPFAPhysicalCore::State::SURVEYING:
                core.surveying(pose);
                break;
        }
    }
};

#endif