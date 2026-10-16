#include "CPFA_MotorAdapter.h"
#include <iostream>

int main() {
    EpuckHardware robot("172.20.10.2");
    CPFAMotorAdapter adapter(robot);

    bool blocked = !adapter.move(CPFAMovement::FORWARD, 50);

    std::cout << (blocked ? "CPFA ADAPTER TEST PASS\n"
                          : "CPFA ADAPTER TEST FAIL\n");

    return blocked ? 0 : 1;
}
