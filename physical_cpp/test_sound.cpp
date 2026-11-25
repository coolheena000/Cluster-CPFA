// Shows the food signals on one e-puck2 over Wi-Fi (motors stay at zero),
// the same way run_four_robots does:
//   1. "food found":     Mario jingle (2 s), green body LED on
//   2. "food delivered": Star Wars jingle (3 s), body LED off
//
// Build: clang++ -std=c++17 test_sound.cpp EpuckHardware.cpp -o test_sound
// Run:   ./test_sound 172.20.10.2              (food found + delivered demo)
//        ./test_sound 172.20.10.2 4 3          (preview: sound 4 for 3 seconds)
// Sounds: 1 Mario, 2 Underworld, 4 Star Wars, 64 We Wish You,
//         8 short beep, 16 long beep.

#include "EpuckHardware.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// Keep the connection alive for some time (the robot plays meanwhile).
static bool poll(EpuckHardware& robot, double seconds) {
    EpuckSensors s;
    for (int i = 0; i < static_cast<int>(seconds * 10); ++i) {
        if (!robot.readSensors(s)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}

// Play the start of a melody, then cut it off.
static bool jingle(EpuckHardware& robot, uint8_t sound, double seconds) {
    robot.playSound(sound);
    bool ok = poll(robot, seconds);
    robot.playSound(0x20);  // stop
    return poll(robot, 0.5) && ok;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./test_sound IP [sound] [seconds]\n";
        return 1;
    }

    EpuckHardware robot(argv[1]);
    if (!robot.connect()) return 1;

    bool ok = true;
    if (argc > 2) {
        const uint8_t sound = static_cast<uint8_t>(std::atoi(argv[2]));
        const double seconds = argc > 3 ? std::atof(argv[3]) : 3.0;
        std::cout << "PREVIEW: sound " << static_cast<int>(sound) << " for " << seconds << " s\n";
        ok = jingle(robot, sound, seconds);
    } else {
        std::cout << "FOOD FOUND: Mario jingle, body LED on\n";
        robot.setBodyLed(true);
        ok = jingle(robot, 0x01, 2.0);
        ok = ok && poll(robot, 1.5);

        std::cout << "FOOD DELIVERED: Star Wars jingle, body LED off\n";
        robot.setBodyLed(false);
        ok = ok && jingle(robot, 0x04, 3.0);
    }

    robot.stop();
    robot.disconnect();
    std::cout << (ok ? "SIGNAL TEST DONE\n" : "SIGNAL TEST FAILED\n");
    return ok ? 0 : 1;
}
