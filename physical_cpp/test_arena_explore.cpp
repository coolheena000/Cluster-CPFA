// Arena test without overhead camera: all robots drive at the same time and
// avoid walls / each other with Carlos's collision maneuver (CPFA_CollisionManeuver.h).
// Shows: Wi-Fi to several robots at once, live IR sensors, motors, avoidance.
//
// Build: clang++ -std=c++17 test_arena_explore.cpp EpuckHardware.cpp -o test_arena_explore
// Run:   ./test_arena_explore 60 172.20.10.2 172.20.10.3 172.20.10.5
//        (first number = seconds, then robot IPs)
// Stop:  Ctrl+C (all robots get STOP)

#include "CPFA_CollisionManeuver.h"
#include "CPFA_EpuckSensorGeometry.h"
#include "CPFA_MotorAdapter.h"
#include "CPFA_PhysicalCollision.h"
#include "EpuckHardware.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

static volatile std::sig_atomic_t gStop = 0;
static void onSignal(int) { gStop = 1; }

static const char* moveName(CPFAMovement m) {
    switch (m) {
        case CPFAMovement::STOP: return "STOP";
        case CPFAMovement::LEFT: return "LEFT";
        case CPFAMovement::RIGHT: return "RIGHT";
        case CPFAMovement::FORWARD: return "FORWARD";
        case CPFAMovement::BACK: return "BACK";
    }
    return "?";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./test_arena_explore SECONDS IP1 [IP2 IP3 IP4]\n";
        return 1;
    }
    std::signal(SIGINT, onSignal);

    const double maxSeconds = std::atof(argv[1]);
    const int16_t forwardSpeed = 200;  // steps/s, about 2.6 cm/s
    const int ticksPerSecond = 8;

    struct Robot {
        std::string ip;
        std::unique_ptr<EpuckHardware> hw;
        std::unique_ptr<CPFAMotorAdapter> motors;
        bool alive = false;
        CPFACollisionManeuver maneuver{200, 150};
    };
    std::vector<Robot> robots;

    for (int a = 2; a < argc; ++a) {
        Robot r;
        r.ip = argv[a];
        r.hw = std::make_unique<EpuckHardware>(r.ip);
        r.alive = r.hw->connect();
        if (r.alive) r.motors = std::make_unique<CPFAMotorAdapter>(*r.hw);
        else std::cout << r.ip << " NOT CONNECTED, skipped\n";
        robots.push_back(std::move(r));
    }

    CPFAPhysicalCollision collision;
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    unsigned long tick = 0;

    std::cout << "ARENA TEST STARTED (" << maxSeconds << " s). Ctrl+C to stop.\n";

    while (!gStop) {
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (t > maxSeconds) break;

        // Send speeds + sensor request to every robot, then read every reply.
        for (auto& r : robots)
            if (r.alive && !r.hw->sendRequest()) r.alive = false;

        for (std::size_t i = 0; i < robots.size(); ++i) {
            auto& r = robots[i];
            if (!r.alive) continue;

            EpuckSensors s;
            if (!r.hw->receiveSensors(s)) {
                std::cout << r.ip << " LOST, sending STOP\n";
                r.hw->stop();
                r.alive = false;
                continue;
            }

            const auto d = collision.calculate(s, kEpuckSensorAngles, true);
            CPFAMovement move = CPFAMovement::FORWARD;
            int16_t speed = forwardSpeed;
            r.maneuver.update(d, 1.0 / ticksPerSecond, move, speed);
            r.motors->move(move, speed);

            if (tick % ticksPerSecond == 0) {
                std::printf("R%zu %-13s %-7s IR:", i, r.ip.c_str(), moveName(move));
                for (auto v : s.proximity) std::printf(" %4u", v);
                std::printf("\n");
            }
        }

        bool any = false;
        for (auto& r : robots) any = any || r.alive;
        if (!any) break;

        ++tick;
        next += std::chrono::milliseconds(1000 / ticksPerSecond);
        std::this_thread::sleep_until(next);
    }

    std::cout << "STOPPING ALL ROBOTS\n";
    for (std::size_t i = 0; i < robots.size(); ++i) {
        auto& r = robots[i];
        if (!r.hw) continue;
        const bool ok = r.hw->stop();
        r.hw->disconnect();
        std::printf("R%zu %s: %s, obstacle turns: %d\n", i, r.ip.c_str(),
                    ok ? "STOP SENT" : "STOP FAILED - switch off!", r.maneuver.count());
    }
    return 0;
}
