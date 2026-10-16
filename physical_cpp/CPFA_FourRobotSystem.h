
#ifndef CPFA_FOUR_ROBOT_SYSTEM_H
#define CPFA_FOUR_ROBOT_SYSTEM_H

#include "CPFA_PhysicalIntegration.h"
#include <array>
#include <memory>
#include <vector>
#include <string>

class CPFAFourRobotSystem {
public:
    static constexpr std::size_t RobotCount = 4;

    struct RobotUnit {
        EpuckHardware hardware;
        CPFAPhysicalBridge bridge;
        CPFAPhysicalController controller;
        CPFAPhysicalIntegration integration;

        explicit RobotUnit(const std::string& ip)
            : hardware(ip),
              bridge(hardware),
              controller(),
              integration(bridge, controller) {}
    };

    explicit CPFAFourRobotSystem(
        const std::array<std::string, RobotCount>& ips
    ) {
        for (const auto& ip : ips) {
            robots_.push_back(std::make_unique<RobotUnit>(ip));
            robots_.back()->controller.sharedPheromones = &sharedPheromones_;
        }
    }
bool sharedPheromonesConnected() const {
    if (robots_.size() != RobotCount) return false;

    for (const auto& robot : robots_) {
        if (robot->controller.sharedPheromones != &sharedPheromones_)
            return false;
    }

    return true;
}

    void updateRobotPose(
        std::size_t robotIndex,
        double x,
        double y,
        double heading
    ) {
        if (robotIndex >= robots_.size()) return;

        robots_[robotIndex]->bridge.updatePose(
            x, y, heading
        );
    }
CPFAPose getRobotPose(std::size_t robotIndex) const {
    if (robotIndex >= robots_.size()) {
        return CPFAPose{};
    }

    return robots_[robotIndex]->bridge.getPose();
}
CPFAMovement decideRobotMovement(
    std::size_t robotIndex,
    bool anglesVerified
) {
    if (robotIndex >= robots_.size()) {
        return CPFAMovement::STOP;
    }

    return robots_[robotIndex]->integration.decide(
        anglesVerified
    );
}

    void stepRobotCPFA(
        std::size_t robotIndex,
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
        if (robotIndex >= robots_.size()) return;

        robots_[robotIndex]->integration.stepCPFA(
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
        auto& core = robots_[robotIndex]->controller.core;
        if (foodDetected && core.isHoldingFood &&
    core.updateFidelity) {
    setRobotFidelity(robotIndex, core.siteFidelityPosition);
}

if (core.isGivingUpSearch &&
    !core.updateFidelity &&
    fidelityActive_[robotIndex]) {
    removeRobotFidelity(robotIndex);
}
    }
CPFAPhysicalCore::State getRobotState(std::size_t robotIndex) const {
    if (robotIndex >= robots_.size()) {
        return CPFAPhysicalCore::State::DEPARTING;
    }

    return robots_[robotIndex]->controller.core.state;
}
void configureRobotDeparture(
    std::size_t robotIndex,
    CPFATarget target,
    double probability
) {
    if (robotIndex >= robots_.size()) return;

    auto& core = robots_[robotIndex]->controller.core;
    core.target = target;
    core.probabilityOfSwitchingToSearching = probability;
}
bool updateRobotSensors(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) {
        return false;
    }

    return robots_[robotIndex]->bridge.updateSensors();
}
bool connectRobot(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) {
        return false;
    }

    return robots_[robotIndex]->hardware.connect();
}

void disconnectRobot(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) {
        return;
    }

    robots_[robotIndex]->hardware.disconnect();
}
void setRobotFidelity(
    std::size_t robotIndex,
    CPFATarget position
) {
    if (robotIndex >= robots_.size()) return;

    sharedFidelityPositions_[robotIndex] = position;
    fidelityActive_[robotIndex] = true;
    robots_[robotIndex]->controller.core.siteFidelityPosition = position;
}
void removeRobotFidelity(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) return;

    fidelityActive_[robotIndex] = false;
}
bool isRobotFidelityActive(std::size_t robotIndex) const {
    if (robotIndex >= robots_.size()) {
        return false;
    }

    return fidelityActive_[robotIndex];
}
void resetRobotForTest(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) return;
    robots_[robotIndex]->controller.core = CPFAPhysicalCore{};
}
void setRobotReturnProbabilityForTest(
    std::size_t robotIndex, double probability) {
    if (robotIndex >= robots_.size()) return;
    robots_[robotIndex]->controller.core.probabilityOfReturningToNest =
        probability;
}
// ---- Physical run support (Carlos CPFA_loop_functions equivalents) ----

// Carlos UpdatePheromoneList(): decay every pheromone, drop inactive ones.
// Call every 0.5 s.
void updatePheromones(double timeSeconds) {
    std::vector<CPFAPhysicalPheromone> active;
    for (auto& p : sharedPheromones_) {
        p.update(timeSeconds);
        if (p.isActive()) active.push_back(p);
    }
    sharedPheromones_ = active;
}

std::size_t pheromoneCount() const {
    return sharedPheromones_.size();
}

CPFAPhysicalCore& robotCore(std::size_t robotIndex) {
    return robots_.at(robotIndex)->controller.core;
}

void invalidateRobotPose(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) return;
    robots_[robotIndex]->bridge.invalidatePose();
}

bool moveRobot(std::size_t robotIndex, CPFAMovement movement, int16_t speed) {
    if (robotIndex >= robots_.size()) return false;
    return robots_[robotIndex]->bridge.move(movement, speed);
}

// Send to all robots first, then receive from all, so one loop
// costs about one Wi-Fi round trip instead of four.
bool sendRobotRequest(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) return false;
    return robots_[robotIndex]->bridge.sendRequest();
}

bool receiveRobotSensors(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) return false;
    return robots_[robotIndex]->bridge.receiveSensors();
}

EpuckSensors getRobotSensors(std::size_t robotIndex) const {
    if (robotIndex >= robots_.size()) return EpuckSensors{};
    return robots_[robotIndex]->bridge.getSensors();
}

bool stopRobot(std::size_t robotIndex) {
    if (robotIndex >= robots_.size()) return false;
    return robots_[robotIndex]->hardware.stop();
}

private:
    std::vector<CPFAPhysicalPheromone> sharedPheromones_;
    std::vector<std::unique_ptr<RobotUnit>> robots_;
    std::array<CPFATarget, RobotCount> sharedFidelityPositions_{};
    std::array<bool, RobotCount> fidelityActive_{};
};

#endif
