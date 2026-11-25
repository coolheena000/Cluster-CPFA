// One-robot navigation test with the overhead camera (no CPFA decisions):
//   1. drive from the nest to the food tag closest to tag 13 (or, with no food
//      near tag 13, to a point just inside the tag 13 corner),
//   2. pick up the food when within 0.13 m (Mario sound, body LED on),
//   3. drive back to the nest and deliver it (Star Wars sound).
//   Other food passed on the way is picked up the same way.
// Uses the same camera, navigation and collision maneuver code as the CPFA run,
// so it shows whether the robot can reach a target and come back without
// getting stuck at a wall.
//
// Build:
//   export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
//   clang++ -std=c++17 test_goto_tag13.cpp EpuckHardware.cpp \
//       $(pkg-config --cflags --libs opencv5) -o test_goto_tag13
// Run:   ./robot_goto13.sh 05359          (finds the robot by MAC, then runs this)
//        ./test_goto_tag13 IP TAG [halfX halfY]
// Stop:  Ctrl+C or Q in the camera window (robot gets STOP)
// Log:   goto13_log.csv (8 rows per second)

#include "CPFA_CollisionManeuver.h"
#include "CPFA_EpuckSensorGeometry.h"
#include "CPFA_MotorAdapter.h"
#include "CPFA_OverheadCamera.h"
#include "CPFA_PhysicalNavigation.h"
#include "EpuckHardware.h"

#include <opencv2/highgui.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// ===================== EDIT FOR YOUR ARENA =====================
// Half the distance between the corner tag centres, metres. Default: the
// temporary square arena, 59 in x 59 in (1.50 m) between the corner tags.
static const double kDefaultHalfX = 0.749;
static const double kDefaultHalfY = 0.749;
static const double kCornerMargin = 0.15;   // target sits this far inside tag 13
static const double kNestRadius = 0.25;
static const double kFoodRadius = 0.13;     // Carlos FoodDistanceTolerance
static const double kFoodMergeRadius = 0.10;
static const int kTicksPerSecond = 8;
static const int16_t kForwardSpeed = 300;
static const int16_t kTurnSpeed = 120;
static const double kPoseTimeout = 0.5;     // s without camera pose -> STOP
static const double kMaxSeconds = 180.0;
static const int kCameraIndex = 0;
// ===============================================================

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
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (argc < 3) {
        std::cerr << "Usage: ./test_goto_tag13 IP TAG [corner] [halfX halfY]   (TAG = robot tag 1-4)\n"
                     "Easier: ./robot_goto13.sh 05367 corner\n";
        return 1;
    }
    const std::string ip = argv[1];
    const int tag = std::atoi(argv[2]);
    // Optional: "corner" = go to the tag 13 corner even if food is nearby;
    // two numbers = arena half sizes in metres.
    bool cornerOnly = false;
    std::vector<double> numbers;
    for (int a = 3; a < argc; ++a) {
        if (std::string(argv[a]) == "corner") cornerOnly = true;
        else numbers.push_back(std::atof(argv[a]));
    }
    const double halfX = numbers.size() >= 2 ? numbers[0] : kDefaultHalfX;
    const double halfY = numbers.size() >= 2 ? numbers[1] : kDefaultHalfY;
    if (tag < 1 || tag > CPFAOverheadCamera::kRobotCount) {
        std::cerr << "Robot tag must be 1 to " << CPFAOverheadCamera::kRobotCount << "\n";
        return 1;
    }
    const int robotIndex = tag - CPFAOverheadCamera::kFirstRobotTag;

    // Tag 13 is the corner (-halfX, -halfY) in CPFA_OverheadCamera.h.
    const CPFATarget corner13{-halfX + kCornerMargin, -halfY + kCornerMargin};
    const CPFATarget nest{0.0, 0.0};
    std::printf("Arena between corner tags: %.2f m x %.2f m\n", 2 * halfX, 2 * halfY);
    std::printf("Target near tag 13: (%.2f, %.2f), then back to the nest (0, 0)\n",
                corner13.x, corner13.y);

    // ---- camera: calibrate on the four corner tags ----
    CPFAOverheadCamera overhead(halfX, halfY);
    if (!overhead.open(kCameraIndex)) {
        std::cerr << "Overhead camera " << kCameraIndex << " not available\n";
        return 1;
    }
    std::cout << "Waiting for corner tags 10, 11, 12, 13...\n";
    for (int i = 0; i < 300 && !overhead.isCalibrated() && !gStop; ++i) {
        overhead.update();
        cv::imshow("Go to tag 13", overhead.debugFrame());
        cv::waitKey(1);
    }
    if (!overhead.isCalibrated()) {
        std::cerr << "Camera NOT calibrated: corner tags 10-13 not all visible\n";
        return 1;
    }
    std::cout << "Camera calibrated\n";

    // ---- food tags, merged over a few seconds like the CPFA run ----
    std::vector<CPFATarget> food;
    const auto scanStart = std::chrono::steady_clock::now();
    while (!gStop && std::chrono::duration<double>(
               std::chrono::steady_clock::now() - scanStart).count() < 3.0) {
        overhead.update();
        for (const auto& f : overhead.foodSeen()) {
            const bool known = std::any_of(food.begin(), food.end(), [&](const CPFATarget& g) {
                return std::hypot(g.x - f.x, g.y - f.y) < kFoodMergeRadius;
            });
            if (!known) food.push_back({f.x, f.y});
        }
        cv::imshow("Go to tag 13", overhead.debugFrame());
        cv::waitKey(1);
    }
    std::cout << "Food tags seen: " << food.size() << "\n";

    // Go to the food closest to tag 13, if one is within 0.8 m of the corner.
    CPFATarget goal = corner13;
    bool goalIsFood = false;
    double best = 0.8;
    for (const auto& f : food) {
        if (cornerOnly) break;
        const double d = std::hypot(f.x - corner13.x, f.y - corner13.y);
        if (d < best) { best = d; goal = f; goalIsFood = true; }
    }
    if (goalIsFood)
        std::printf("Going to the food closest to tag 13: (%.2f, %.2f)\n", goal.x, goal.y);
    else
        std::printf("No food near tag 13; going to the corner point (%.2f, %.2f)\n", goal.x, goal.y);

    // ---- robot ----
    EpuckHardware robot(ip);
    if (!robot.connect()) return 1;
    CPFAMotorAdapter motors(robot);
    CPFACollisionManeuver maneuver(kForwardSpeed, kTurnSpeed);

    std::ofstream log("goto13_log.csv");
    log << "time_s,phase,x_m,y_m,heading_rad,pose_valid,target_x,target_y,dist_m,move,"
           "holding_food,ir0,ir1,ir2,ir3,ir4,ir5,ir6,ir7\n";

    enum class Phase { TO_TAG13, TO_NEST, DONE };
    Phase phase = Phase::TO_TAG13;
    bool holding = false;
    int delivered = 0;
    double reachedTag13At = -1.0;
    double lastSeen = -1e9;
    CPFAPose pose{};
    EpuckSensors sensors{};
    double soundStopAt = 0.0;

    const auto start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1000 / kTicksPerSecond);
    auto nextTick = start;
    unsigned long tick = 0;
    std::cout << "GO TO TAG 13 STARTED (Ctrl+C or Q to stop)\n";

    while (!gStop && phase != Phase::DONE) {
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (now > kMaxSeconds) {
            std::cout << "Time limit reached\n";
            break;
        }

        // 1. Camera pose; stale pose = invalid = STOP.
        if (overhead.update()) {
            const auto p = overhead.getPose(robotIndex);
            if (p.seen) {
                pose = {p.x, p.y, p.heading, true};
                lastSeen = now;
            }
        }
        if (now - lastSeen > kPoseTimeout) pose.valid = false;
        cv::imshow("Go to tag 13", overhead.debugFrame());
        if ((cv::waitKey(1) & 0xFF) == 'q') gStop = 1;

        const CPFATarget target = phase == Phase::TO_TAG13 ? goal : nest;
        const double dist = std::hypot(target.x - pose.x, target.y - pose.y);

        // 2. Food on the way (same 0.13 m rule as ARGoS).
        if (pose.valid && !holding) {
            for (std::size_t f = 0; f < food.size(); ++f) {
                if (std::hypot(pose.x - food[f].x, pose.y - food[f].y) < kFoodRadius) {
                    holding = true;
                    food.erase(food.begin() + static_cast<long>(f));
                    robot.playSound(0x01);
                    soundStopAt = now + 2.0;
                    std::printf("%.1f s: FOUND FOOD at (%.2f, %.2f)\n", now, pose.x, pose.y);
                    break;
                }
            }
        }

        // 3. Phase changes.
        if (pose.valid && phase == Phase::TO_TAG13 &&
            (dist < 0.05 || (goalIsFood && holding))) {
            reachedTag13At = now;
            phase = Phase::TO_NEST;
            std::printf("%.1f s: REACHED THE %s, going back to the nest\n", now,
                        goalIsFood ? "FOOD NEAR TAG 13" : "TAG 13 CORNER");
        } else if (pose.valid && phase == Phase::TO_NEST &&
                   std::hypot(pose.x, pose.y) < kNestRadius) {
            if (holding) {
                ++delivered;
                holding = false;
                robot.playSound(0x04);
                soundStopAt = now + 3.0;
            }
            phase = Phase::DONE;
            std::printf("%.1f s: BACK IN THE NEST, food delivered: %d\n", now, delivered);
        }
        if (soundStopAt > 0.0 && now >= soundStopAt) {
            robot.playSound(0x20);
            soundStopAt = 0.0;
        }
        robot.setBodyLed(holding);

        // 4. Movement: collision maneuver first, then navigation to the target.
        CPFAMovement movement = CPFAMovement::STOP;
        int16_t speed = 0;
        if (pose.valid && phase != Phase::DONE) {
            const auto collision = CPFAPhysicalCollision{}.calculate(
                sensors, kEpuckSensorAngles, sensors.valid);
            if (!maneuver.update(collision, 1.0 / kTicksPerSecond, movement, speed)) {
                movement = CPFAPhysicalNavigation{}.calculate(
                    pose, target, phase == Phase::TO_NEST).movement;
                speed = movement == CPFAMovement::FORWARD ? kForwardSpeed : kTurnSpeed;
            }
        }
        motors.move(movement, speed);

        if (tick % kTicksPerSecond == 0) {
            std::printf("%5.1f s %-8s pose=%s(%.2f,%.2f,%.2f) to target %.2f m move=%s%s\n",
                        now, phase == Phase::TO_TAG13 ? "TO_TAG13" : "TO_NEST",
                        pose.valid ? "" : "LOST", pose.x, pose.y, pose.heading, dist,
                        moveName(movement), holding ? " [food]" : "");
        }
        log << now << ',' << (phase == Phase::TO_TAG13 ? "TO_TAG13" : "TO_NEST") << ','
            << pose.x << ',' << pose.y << ',' << pose.heading << ',' << pose.valid << ','
            << target.x << ',' << target.y << ',' << dist << ',' << moveName(movement) << ','
            << holding;
        for (auto v : sensors.proximity) log << ',' << v;
        log << '\n' << std::flush;

        // 5. Send speeds + sensor request, read the reply.
        if (!robot.sendRequest() || !robot.receiveSensors(sensors)) {
            std::cerr << "Robot lost (Wi-Fi). Sending STOP.\n";
            break;
        }

        ++tick;
        nextTick += period;
        std::this_thread::sleep_until(nextTick);
    }

    // Wheels keep the last speed after the socket closes: STOP several times.
    for (int repeat = 0; repeat < 5; ++repeat) {
        robot.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    robot.disconnect();

    const double total = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::printf("RESULT: reached the %s: %s", goalIsFood ? "food near tag 13" : "tag 13 corner",
                reachedTag13At >= 0 ? "YES" : "NO");
    if (reachedTag13At >= 0) std::printf(" (after %.0f s)", reachedTag13At);
    std::printf(", back in nest: %s, food delivered: %d, time: %.0f s\n",
                phase == Phase::DONE ? "YES" : "NO", delivered, total);
    return 0;
}
