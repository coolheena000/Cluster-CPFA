#include "CPFA_PhysicalCore.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    CPFAPhysicalCore core;

    core.state = CPFAPhysicalCore::State::SURVEYING;

    CPFAPose pose{};
    pose.valid = true;
    pose.x = 0.0;
    pose.y = 0.0;

    constexpr double pi = 3.141592653589793;

    // Carlos: five survey headings (0, 90, 180, 270, 360).
    for (int i = 0; i <= 4; ++i) {
        pose.heading = i * pi / 2.0;
        core.surveying(pose);

        assert(core.state == CPFAPhysicalCore::State::SURVEYING);
        assert(core.surveyCount == static_cast<unsigned int>(i + 1));
    }

    // After surveying, transition to returning.
    core.surveying(pose);

    assert(core.state == CPFAPhysicalCore::State::RETURNING);
    assert(core.surveyCount == 0);
    assert(core.headingToNest());

    std::cout << "SURVEYING TEST PASS\n";
    return 0;
}