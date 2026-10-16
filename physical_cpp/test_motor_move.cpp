// One-robot motor test without camera.
// Sequence: FORWARD, STOP, LEFT, STOP, RIGHT, STOP, BACK, STOP (1 s each).
// Stops early if something is close in front (IR0 / IR7) or on Ctrl+C.
//
// Build: clang++ -std=c++17 test_motor_move.cpp EpuckHardware.cpp -o test_motor_move
// Run:   ./test_motor_move 172.20.10.2

#include "EpuckHardware.h"
#include "CPFA_MotorAdapter.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t gStop = 0;
static void onSignal(int) { gStop = 1; }

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);

    const std::string ip = argc > 1 ? argv[1] : "172.20.10.2";
    const int16_t speed = 200;               // steps/s, about 2.6 cm/s
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
        for (int i = 0; i < 10 && !gStop; ++i) {
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
