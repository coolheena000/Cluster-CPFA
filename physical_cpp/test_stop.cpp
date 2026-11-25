#include "EpuckHardware.h"
#include <iostream>

// Run: ./test_stop [robot IP]   (default 172.20.10.2)
int main(int argc, char** argv) {
    EpuckHardware robot(argc > 1 ? argv[1] : "172.20.10.2");

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
