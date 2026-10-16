#include "EpuckHardware.h"
#include <iostream>
int main() {
    EpuckHardware robot("172.20.10.2");
    if (!robot.connect()) {
        std::cerr << "Wi-Fi connection failed\n";
        return 1;
    }
    EpuckSensors sensors;
    bool ok = robot.readSensors(sensors);
    robot.disconnect();
    if (!ok || !sensors.valid) {
        std::cerr << "Sensor reading failed\n";
        return 1;
    }
    std::cout << "PROXIMITY:";
    for (auto value : sensors.proximity)
        std::cout << " " << value;
    std::cout << "\nSENSOR TEST SUCCESS\n";
    return 0;
}
