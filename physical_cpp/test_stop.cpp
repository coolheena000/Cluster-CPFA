#include "EpuckHardware.h"
#include <iostream>

int main() {
    EpuckHardware robot("172.20.10.2");

    if (!robot.connect()) {
        std::cerr << "CONNECTION FAILED\n";
        return 1;
    }

    bool ok = robot.stop();
    robot.disconnect();

    std::cout << (ok ? "ZERO COMMAND SENT\n"
                     : "ZERO COMMAND FAILED\n");
    return ok ? 0 : 1;
}
