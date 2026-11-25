// One-robot motor test without camera.
// Sequence: FORWARD, STOP, LEFT, STOP, RIGHT, STOP, BACK, STOP (1 s each).
// Stops early if something is close in front (IR0 / IR7) or on Ctrl+C.
//
// Build: clang++ -std=c++17 test_motor_move.cpp EpuckHardware.cpp -o test_motor_move
// Run:   ./test_motor_move IP [seconds per step] [speed]
//        Easier: ./robot_motor.sh 05367 2 300   (finds the robot by MAC first)

#include "EpuckHardware.h"
#include "CPFA_MotorAdapter.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t gStop = 0;
static void onSignal(int) { gStop = 1; }

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);

    const std::string ip = argc > 1 ? argv[1] : "172.20.10.2";
    const double seconds = argc > 2 ? std::atof(argv[2]) : 1.0;  // per step
    const int16_t speed = static_cast<int16_t>(argc > 3 ? std::atoi(argv[3]) : 200);  // steps/s, max 400
    // e-puck2: 1000 steps = 0.1288 m, wheel axle 0.053 m.
    const double metres = speed * 0.1288 / 1000.0 * seconds;
    std::printf("Each step %.1f s at speed %d. Expected: FORWARD/BACK %.1f cm, LEFT/RIGHT %.0f degrees\n",
                seconds, speed, metres * 100.0, 2.0 * metres / 0.053 * 180.0 / 3.141592653589793);
    const uint16_t frontObstacle = 300;      // IR0 / IR7 value that means "too close"

    EpuckHardware robot(ip);
    if (!robot.connect()) return 1;
    CPFAMotorAdapter motors(robot);

    const struct { CPFAMovement move; const char* name; } steps[] = {
        {CPFAMovement::FORWARD, "FORWARD"}, {CPFAMovement::STOP, "STOP"},
        {CPFAMovement::LEFT, "LEFT"},       {CPFAMovement::STOP, "STOP"},
        {CPFAMovement::RIGHT, "RIGHT"},     {CPFAMovement::STOP, "STOP"},
        {CPFAMovement::BACK, "BACK"},       {CPFAMovement::STOP, "STOP"},
    };

    bool ok = true;
    for (const auto& step : steps) {
        if (gStop) break;
        std::cout << step.name << "\n";

        if (!motors.move(step.move, speed)) {
            std::cout << "Motor command refused (safety guard in EpuckHardware.cpp)\n";
            ok = false;
            break;
        }

        // Speeds are sent with each sensor request, 10 times per second.
        for (int i = 0; i < static_cast<int>(seconds * 10 + 0.5) && !gStop; ++i) {
            EpuckSensors s;
            if (!robot.readSensors(s)) { ok = false; gStop = 1; break; }

            if (step.move == CPFAMovement::FORWARD &&
                (s.proximity[0] > frontObstacle || s.proximity[7] > frontObstacle)) {
                std::cout << "Obstacle in front -> STOP\n";
                gStop = 1;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    const bool stopped = robot.stop();
    robot.disconnect();
    std::cout << (stopped ? "FINAL STOP SENT\n" : "FINAL STOP FAILED - switch the robot off!\n");
    std::cout << (ok && !gStop ? "MOTOR MOVE TEST DONE\n" : "MOTOR MOVE TEST ENDED EARLY\n");
    return ok && stopped ? 0 : 1;
}
