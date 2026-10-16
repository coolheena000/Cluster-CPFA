// Checks Wi-Fi communication with every robot: connect + read sensors.
// Motors stay at zero speed.
//
// Build: clang++ -std=c++17 test_four_connections.cpp EpuckHardware.cpp -o test_four_connections
// Run:   ./test_four_connections 172.20.10.2 172.20.10.3 172.20.10.4 172.20.10.5

#include "EpuckHardware.h"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./test_four_connections IP1 [IP2 IP3 IP4]\n";
        return 1;
    }

    int okCount = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string ip = argv[i];
        std::cout << "Robot " << i - 1 << " (" << ip << "): ";

        EpuckHardware robot(ip);
        if (!robot.connect()) {
            std::cout << "NOT CONNECTED\n";
            continue;
        }

        EpuckSensors s;
        const bool ok = robot.readSensors(s) && s.valid;
        robot.disconnect();

        if (!ok) {
            std::cout << "CONNECTED but no sensor data (selector 15? reset robot)\n";
            continue;
        }

        std::cout << "OK  proximity:";
        for (auto v : s.proximity) std::cout << " " << v;
        std::cout << "\n";
        ++okCount;
    }

    std::cout << okCount << " of " << argc - 1 << " robots OK\n";
    return okCount == argc - 1 ? 0 : 1;
}
