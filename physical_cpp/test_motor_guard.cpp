#include "EpuckHardware.h"
#include <iostream>

int main() {
    EpuckHardware robot("172.20.10.2");

    bool blocked = !robot.setWheelSpeeds(50, 50);

    std::cout << (blocked ? "MOTOR SAFETY GUARD PASS\n"
                          : "MOTOR SAFETY GUARD FAIL\n");

    return blocked ? 0 : 1;
}
