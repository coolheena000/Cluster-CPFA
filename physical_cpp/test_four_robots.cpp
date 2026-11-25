
#include "CPFA_FourRobotSystem.h"
#include <cassert>
#include <iostream>

int main() {
    std::array<std::string, 4> ips = {
        "127.0.0.1",
        "127.0.0.2",
        "127.0.0.3",
        "127.0.0.4"
    };

    CPFAFourRobotSystem system(ips);
assert(system.sharedPheromonesConnected());
// Offline test: independent localization for four robots.
system.updateRobotPose(0, 0.5, 0.5, 0.0);

assert(system.getRobotPose(0).valid);
assert(system.getRobotPose(0).x == 0.5);

assert(!system.getRobotPose(1).valid);
assert(!system.getRobotPose(2).valid);
assert(!system.getRobotPose(3).valid);
// Offline test: all four robots must STOP without verified sensors.
for (std::size_t i = 0; i < 4; ++i) {
    assert(system.decideRobotMovement(i, false) == CPFAMovement::STOP);
}
// Offline test: independent CPFA state updates.
system.updateRobotPose(0, 0.5, 0.5, 0.0);

system.stepRobotCPFA(
    0,
    true, false, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 0.5
);
// Force Robot 1 to enter SEARCHING using existing CPFA logic.
system.configureRobotDeparture(0, CPFATarget{0.5, 0.5}, 1.0);

system.stepRobotCPFA(
    0,
    true, false, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 1.0
);
assert(system.getRobotState(0) == CPFAPhysicalCore::State::SEARCHING);
assert(system.getRobotState(1) == CPFAPhysicalCore::State::DEPARTING);
assert(system.getRobotState(2) == CPFAPhysicalCore::State::DEPARTING);
assert(system.getRobotState(3) == CPFAPhysicalCore::State::DEPARTING);
// Robot 1 detects food and enters SURVEYING.
system.stepRobotCPFA(
    0,
    true, true, false,
    0.5, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 1.5
);

assert(system.getRobotState(0) == CPFAPhysicalCore::State::SURVEYING);

for (std::size_t i = 1; i < 4; ++i) {
    assert(system.getRobotState(i) == CPFAPhysicalCore::State::DEPARTING);
} 

// Offline safety test: invalid robot index must be rejected.
assert(!system.updateRobotSensors(4));
// Test independent site-fidelity entries.
system.setRobotFidelity(0, CPFATarget{0.5, 0.5});
system.setRobotFidelity(1, CPFATarget{0.8, 0.8});

assert(system.isRobotFidelityActive(0));
assert(system.isRobotFidelityActive(1));

system.removeRobotFidelity(0);

assert(!system.isRobotFidelityActive(0));
assert(system.isRobotFidelityActive(1));
// Test automatic fidelity removal when robot abandons search.
system.resetRobotForTest(0);
system.setRobotReturnProbabilityForTest(0, 1.0);
system.configureRobotDeparture(0, CPFATarget{0.5, 0.5}, 1.0);
system.setRobotFidelity(0, CPFATarget{0.5, 0.5});

system.updateRobotPose(0, 0.5, 0.5, 0.0);
system.stepRobotCPFA(
    0, true, false, false,
    0.0, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 1.5
);

assert(system.getRobotState(0) ==
       CPFAPhysicalCore::State::SEARCHING);
// Within 5 cm of the 8 cm search step, so a new search decision is made.
system.updateRobotPose(0, 0.60, 0.5, 0.0);
system.stepRobotCPFA(
    0, true, false, false,
    0.0, 0.0, 0.5, 0.5,
    0.5, 0.5, 0.5,
    CPFATarget{}, 2.0
);
assert(system.getRobotState(0) == CPFAPhysicalCore::State::RETURNING);
assert(!system.isRobotFidelityActive(0));
// Verify automatic fidelity registration after food detection.
system.resetRobotForTest(2);
system.updateRobotPose(2, 0.4, 0.6, 0.0);
system.configureRobotDeparture(2, CPFATarget{0.4, 0.6}, 1.0);

system.stepRobotCPFA(
    2, true, false, false,
    0.0, 0.0, 0.4, 0.6,
    0.5, 0.5, 0.5,
    CPFATarget{}, 3.0
);

assert(system.getRobotState(2) ==
       CPFAPhysicalCore::State::SEARCHING);

system.stepRobotCPFA(
    2, true, true, false,
    0.0, 0.0, 0.4, 0.6,
    0.5, 0.5, 0.5,
    CPFATarget{}, 3.5
);

assert(system.getRobotState(2) ==
       CPFAPhysicalCore::State::SURVEYING);
assert(system.isRobotFidelityActive(2));
std::cout << "FOUR ROBOT SYSTEM aCONSTRUCTION PASS\n";
return 0;
} 

