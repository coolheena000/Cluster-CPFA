// One robot, no overhead camera: drive forward, and when an obstacle is
// in front, STOP, take a photo with the e-puck camera, turn left, continue.
// Photos: obstacle_1.bmp, obstacle_2.bmp, ...
//
// Build: clang++ -std=c++17 test_explore_photo.cpp EpuckHardware.cpp -o test_explore_photo
// Run:   ./test_explore_photo 172.20.10.2
//        ./test_explore_photo 172.20.10.2 60 5   (60 seconds, max 5 photos)
// Stop:  Ctrl+C (robot gets STOP)

#include "EpuckHardware.h"
#include "CPFA_MotorAdapter.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

static volatile std::sig_atomic_t gStop = 0;
static void onSignal(int) { gStop = 1; }

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);

    const std::string ip = argc > 1 ? argv[1] : "172.20.10.2";
    const double maxSeconds = argc > 2 ? std::atof(argv[2]) : 30.0;
    const int maxPhotos = argc > 3 ? std::atoi(argv[3]) : 3;

    const int16_t forwardSpeed = 200;   // steps/s, about 2.6 cm/s
    const int16_t turnSpeed = 150;
    const uint16_t frontObstacle = 300; // IR value that means "obstacle close"

    EpuckHardware robot(ip);
    if (!robot.connect()) return 1;
    CPFAMotorAdapter motors(robot);

    const auto start = std::chrono::steady_clock::now();
    auto elapsed = [&] {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
    };

    int photos = 0;
    bool ok = true;

    std::cout << "EXPLORE STARTED (" << maxSeconds << " s, max "
              << maxPhotos << " photos). Ctrl+C to stop.\n";

    while (!gStop && ok && elapsed() < maxSeconds && photos < maxPhotos) {
        if (!motors.move(CPFAMovement::FORWARD, forwardSpeed)) {
            std::cout << "Motor command refused (safety guard in EpuckHardware.cpp)\n";
            break;
        }

        EpuckSensors s;
        if (!robot.readSensors(s)) { ok = false; break; }

        const uint16_t front = std::max({s.proximity[0], s.proximity[7],
                                         s.proximity[1], s.proximity[6]});
        if (front > frontObstacle) {
            std::cout << "OBSTACLE (IR " << front << ") -> STOP, photo\n";
            if (!robot.stop()) { ok = false; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // The first frame after a request can be stale; keep the second.
            std::vector<uint8_t> img;
            if (robot.captureImage(img) && robot.captureImage(img)) {
                const std::string name = "obstacle_" + std::to_string(++photos) + ".bmp";
                if (EpuckHardware::saveImageBmp(name, img))
                    std::cout << "Saved " << name << "\n";
            } else {
                std::cout << "Photo failed\n";
                ok = false;
                break;
            }

            // Turn left away from the obstacle for about 1 s.
            motors.move(CPFAMovement::LEFT, turnSpeed);
            for (int i = 0; i < 10 && !gStop; ++i) {
                if (!robot.readSensors(s)) { ok = false; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const bool stopped = robot.stop();
    robot.disconnect();
    std::cout << (stopped ? "FINAL STOP SENT\n" : "FINAL STOP FAILED - switch the robot off!\n");
    std::cout << "Photos taken: " << photos << "\n";
    return ok && stopped ? 0 : 1;
}
