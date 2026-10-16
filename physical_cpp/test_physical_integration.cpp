
#include "CPFA_PhysicalIntegration.h"
#include <cassert>
#include <iostream>

int main() {
    // Construct hardware interface without connecting.
    EpuckHardware robot("127.0.0.1");
    CPFAPhysicalBridge bridge(robot);
    CPFAPhysicalController controller;
    CPFAPhysicalIntegration integration(bridge, controller);

    // No camera pose and no valid sensor readings.
    // The controller must return STOP.
    CPFAMovement decision = integration.decide(false);
    
    assert(decision == CPFAMovement::STOP);
    assert(!integration.executeStop());
    // Invalid camera pose must not advance CPFA state.
integration.stepCPFA(
    true, false, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 0.0
);

assert(controller.core.state == CPFAPhysicalCore::State::DEPARTING);
// Offline test: valid camera pose.
bridge.updatePose(0.0, 0.0, 0.0);

controller.core.target = {1.0, 0.0};
controller.core.probabilityOfSwitchingToSearching = 1.0;

integration.stepCPFA(
    true, false, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 0.5
);

assert(controller.core.state == CPFAPhysicalCore::State::SEARCHING);
// Offline test: food detected during SEARCHING.
integration.stepCPFA(
    true, true, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 1.0
);

assert(controller.core.state == CPFAPhysicalCore::State::SURVEYING);
assert(controller.core.isHoldingFood);
// Offline test: surveying completed.
controller.core.surveyCount = 5;

integration.stepCPFA(
    true, false, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 1.5
);

assert(controller.core.state == CPFAPhysicalCore::State::RETURNING);
// Offline test: robot returns to nest.
std::vector<CPFAPhysicalPheromone> pheromones;
controller.sharedPheromones = &pheromones;

integration.stepCPFA(
    true, false, true,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{0.5, 0.5}, 2.0
);

assert(controller.core.state == CPFAPhysicalCore::State::DEPARTING);
assert(!controller.core.isHoldingFood);
assert(controller.core.numTargetsCollected == 1);
    std::cout << "PHYSICAL INTEGRATION SAFETY TEST PASS\n";
    return 0;
}
