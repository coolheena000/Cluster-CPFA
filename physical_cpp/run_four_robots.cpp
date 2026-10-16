// Runs CPFA on four physical e-puck2 robots.
// Plays the role of Carlos's CPFA_loop_functions + ARGoS main loop:
//   - robot poses come from the overhead camera (CPFA_OverheadCamera.h)
//   - food is a virtual list of arena positions (like LoopFunctions->FoodList)
//   - shared pheromones decay every 0.5 s (UpdatePheromoneList)
//
// Build:
//   export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
//   clang++ -std=c++17 run_four_robots.cpp EpuckHardware.cpp \
//       $(pkg-config --cflags --libs opencv5) -o run_four_robots
// Run:   ./run_four_robots
// Stop:  Ctrl+C or Q in the camera window (all robots get a STOP command)

#include "CPFA_CollisionManeuver.h"
#include "CPFA_FourRobotSystem.h"
#include "CPFA_OverheadCamera.h"

#include <opencv2/highgui.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <random>
#include <thread>

// ===================== EDIT FOR YOUR ARENA =====================
// Robot i carries AprilTag tag36h11 ID i+1 (1..4); corners 10..13; food cubes ID 0.
static const std::array<std::string, 4> kRobotIps = {
    "172.20.10.2",  // e-puck2_05367, tag 1
    "172.20.10.4",  // e-puck2_05333, tag 2
    "172.20.10.5",  // e-puck2_05359, tag 3
    "172.20.10.7"   // e-puck2_05417, tag 4
};
static const int kRobotPort = 1000;

// Arena is centred on (0,0), metres. x to the right, y up, heading CCW.
// Lab arena: 100 in x 66 in (2.54 m x 1.68 m), measured INSIDE the wood frame.
static const double kArenaHalfX = 1.27;
static const double kArenaHalfY = 0.84;
static const double kRobotRadius = 0.06;  // e-puck2 radius 0.035 + wall margin

// Carlos (methods_and_design.tex): central nest, radius 0.25 m.
static const CPFATarget kNest{0.0, 0.0};
static const double kNestRadius = 0.25;

// Carlos ClusterFoodDistribution(): 4 clusters, ClusterWidthX x ClusterLengthY
// items each, spaced 3 * FoodRadius = 0.15 m. Carlos uses 4 x 4.
// Cluster positions are fixed (not random) so paper dots can be placed.
static const int kClusterWidthX = 4;
static const int kClusterLengthY = 4;
static const double kFoodOffset = 3.0 * 0.05;
static const std::array<CPFATarget, 4> kClusterCenters = {{
    {0.75, 0.45}, {-0.75, 0.45}, {-0.75, -0.45}, {0.75, -0.45}
}};
static std::vector<CPFATarget> gFood;

// Carlos FoodDistanceTolerance = 0.13 m.
static const double kFoodRadius = 0.13;

static const int kTicksPerSecond = 8;
static const int16_t kForwardSpeed = 300;  // steps/s, 1000 = 12.9 cm/s
static const int16_t kTurnSpeed = 120;
static const double kPoseTimeout = 0.5;    // s without camera pose -> STOP
static const int kCameraIndex = 0;         // overhead camera (0 = first camera)
static const bool kShowCameraWindow = true;
static const double kMaxRunSeconds = 600.0;
// ===============================================================

static volatile std::sig_atomic_t gStop = 0;
static void onSignal(int) { gStop = 1; }

struct CameraPose {
    double x = 0, y = 0, heading = 0;
    double receivedAt = -1e9;
};

static const char* stateName(CPFAPhysicalCore::State s) {
    switch (s) {
        case CPFAPhysicalCore::State::DEPARTING: return "DEPARTING";
        case CPFAPhysicalCore::State::SEARCHING: return "SEARCHING";
        case CPFAPhysicalCore::State::RETURNING: return "RETURNING";
        case CPFAPhysicalCore::State::SURVEYING: return "SURVEYING";
    }
    return "?";
}

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

int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // ---- overhead camera: calibrate on the four corner markers first ----
    CPFAOverheadCamera overhead(kArenaHalfX, kArenaHalfY);
    if (!overhead.open(kCameraIndex)) {
        std::cerr << "Overhead camera " << kCameraIndex << " not available\n";
        return 1;
    }
    std::cout << "Waiting for corner tags 10, 11, 12, 13 (tag36h11)...\n";
    for (int i = 0; i < 300 && !overhead.isCalibrated() && !gStop; ++i) {
        overhead.update();
        if (kShowCameraWindow) {
            cv::imshow("CPFA overhead camera", overhead.debugFrame());
            cv::waitKey(1);
        }
    }
    if (!overhead.isCalibrated()) {
        std::cerr << "Camera NOT calibrated: corner tags 10-13 not all visible\n";
        return 1;
    }
    std::cout << "Camera calibrated\n";

    // ---- robots ----
    CPFAFourRobotSystem system(kRobotIps);
    for (std::size_t i = 0; i < 4; ++i) {
        // EpuckHardware takes the port in its constructor; default is 1000.
        (void)kRobotPort;
        if (!system.connectRobot(i)) {
            std::cerr << "Robot " << i << " (" << kRobotIps[i] << ") not connected. Stopping all.\n";
            for (std::size_t j = 0; j < i; ++j) { system.stopRobot(j); system.disconnectRobot(j); }
            return 1;
        }
    }

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> gaussian(0.0, 1.0);

    for (std::size_t i = 0; i < 4; ++i) {
        auto& core = system.robotCore(i);
        core.nest = kNest;
        core.forageMinX = -kArenaHalfX + kRobotRadius;
        core.forageMaxX = kArenaHalfX - kRobotRadius;
        core.forageMinY = -kArenaHalfY + kRobotRadius;
        core.forageMaxY = kArenaHalfY - kRobotRadius;
        core.setRandomSearchLocation(uniform(rng), uniform(rng), uniform(rng));
    }

    // Food = foam cubes the overhead camera sees (any tag that is not a
    // robot or corner tag). Without cubes, fall back to Carlos's
    // ClusterFoodDistribution() with fixed cluster centres.
    for (const auto& f : overhead.foodSeen()) gFood.push_back({f.x, f.y});
    if (!gFood.empty()) {
        std::cout << "FOOD from camera (" << gFood.size() << " cubes):\n";
    } else {
        for (const auto& c : kClusterCenters) {
            const double x0 = c.x - (kClusterWidthX - 1) * kFoodOffset / 2.0;
            const double y0 = c.y - (kClusterLengthY - 1) * kFoodOffset / 2.0;
            for (int j = 0; j < kClusterLengthY; ++j)
                for (int k = 0; k < kClusterWidthX; ++k)
                    gFood.push_back({x0 + k * kFoodOffset, y0 + j * kFoodOffset});
        }
        std::cout << "No food cubes seen. Virtual FOOD (" << gFood.size() << " items):\n";
    }
    for (const auto& f : gFood) std::printf("  (%.2f, %.2f)\n", f.x, f.y);

    std::array<CameraPose, 4> camera{};
    std::array<CPFACollisionManeuver, 4> maneuvers = {{
        {kForwardSpeed, kTurnSpeed}, {kForwardSpeed, kTurnSpeed},
        {kForwardSpeed, kTurnSpeed}, {kForwardSpeed, kTurnSpeed}}};
    std::array<bool, 4> alive{true, true, true, true};
    bool motorsLockedWarned = false;
    unsigned int foodCollected = 0;

    const auto start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1000 / kTicksPerSecond);
    auto nextTick = start;
    unsigned long tick = 0;

    std::cout << "CPFA RUN STARTED (Ctrl+C to stop)\n";

    while (!gStop) {
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (now > kMaxRunSeconds || gFood.empty()) break;

        // 1. Overhead camera -> latest pose of every visible robot marker.
        if (overhead.update()) {
            for (int id = 0; id < 4; ++id) {
                const auto p = overhead.getPose(id);
                if (p.seen) camera[id] = {p.x, p.y, p.heading, now};
            }
        }
        if (kShowCameraWindow) {
            cv::imshow("CPFA overhead camera", overhead.debugFrame());
            if ((cv::waitKey(1) & 0xFF) == 'q') gStop = 1;
        }

        const bool halfSecondTick = (tick % (kTicksPerSecond / 2)) == 0;

        for (std::size_t i = 0; i < 4; ++i) {
            if (!alive[i]) continue;

            // 2. Camera pose -> localization (stale pose = invalid = STOP).
            if (now - camera[i].receivedAt <= kPoseTimeout)
                system.updateRobotPose(i, camera[i].x, camera[i].y, camera[i].heading);
            else
                system.invalidateRobotPose(i);

            const CPFAPose pose = system.getRobotPose(i);
            auto& core = system.robotCore(i);

            // 3. Nest and virtual food checks (Carlos IsInTheNest / SetHoldingFood).
            const bool inNest = pose.valid &&
                std::hypot(pose.x - kNest.x, pose.y - kNest.y) < kNestRadius;

            bool foodDetected = false;
            if (pose.valid && halfSecondTick &&
                core.state == CPFAPhysicalCore::State::SEARCHING && !core.isHoldingFood) {
                for (std::size_t f = 0; f < gFood.size(); ++f) {
                    if (std::hypot(pose.x - gFood[f].x, pose.y - gFood[f].y) < kFoodRadius) {
                        foodDetected = true;
                        gFood.erase(gFood.begin() + static_cast<long>(f));
                        std::cout << "Robot " << i << " FOUND FOOD, " << gFood.size() << " left\n";
                        break;
                    }
                }
            }

            const unsigned int collectedBefore = core.numTargetsCollected;

            // 4. One CPFA step.
            CPFAPhysicalCore randomWall = core;
            randomWall.setRandomSearchLocation(uniform(rng), uniform(rng), uniform(rng));

            system.stepRobotCPFA(
                i, halfSecondTick, foodDetected, inNest,
                uniform(rng), gaussian(rng), uniform(rng), uniform(rng),
                uniform(rng), uniform(rng), uniform(rng),
                randomWall.target, now);

            // Carlos SetLocalResourceDensity(): 1 (this item) + remaining food
            // within SearchRadiusSquared * 2, where SearchRadius = 4 * FoodRadius.
            if (foodDetected && core.isHoldingFood) {
                const double searchRadiusSquared = (4.0 * 0.05) * (4.0 * 0.05);
                double density = 1.0;
                for (const auto& f : gFood) {
                    const double dx = pose.x - f.x, dy = pose.y - f.y;
                    if (dx * dx + dy * dy < searchRadiusSquared * 2.0) density += 1.0;
                }
                core.resourceDensity = density;
                std::cout << "Robot " << i << " resource density " << density << "\n";
            }

            if (core.numTargetsCollected > collectedBefore) {
                ++foodCollected;
                std::cout << "Robot " << i << " DELIVERED FOOD. Total: " << foodCollected << "\n";
            }

            // 5. Collision avoidance (Carlos's maneuver) + navigation -> wheel speeds.
            CPFAMovement movement;
            int16_t speed;
            const auto collision = CPFAPhysicalCollision{}.calculate(
                system.getRobotSensors(i), kEpuckSensorAngles, true);
            if (!pose.valid ||
                !maneuvers[i].update(collision, 1.0 / kTicksPerSecond, movement, speed)) {
                movement = system.decideRobotMovement(i, true);
                speed = movement == CPFAMovement::FORWARD ? kForwardSpeed : kTurnSpeed;
            }

            if (!system.moveRobot(i, movement, speed) &&
                movement != CPFAMovement::STOP && pose.valid && !motorsLockedWarned) {
                std::cout << "NOTE: motor command refused (safety guard in EpuckHardware.cpp)\n";
                motorsLockedWarned = true;
            }

            if (tick % kTicksPerSecond == 0) {
                // Distance to the closest food item, to see why food is (not) picked up.
                double nearestFood = 1e9;
                for (const auto& f : gFood)
                    nearestFood = std::min(nearestFood, std::hypot(pose.x - f.x, pose.y - f.y));

                std::printf("R%zu %-9s pose=%s(%.2f,%.2f,%.2f) target=(%.2f,%.2f) move=%s"
                            " food=%.2fm%s\n",
                    i, stateName(core.state), pose.valid ? "" : "INVALID",
                    pose.x, pose.y, pose.heading, core.target.x, core.target.y,
                    moveName(movement), nearestFood,
                    nearestFood < kFoodRadius ? " (in range)" : "");
            }
        }

        // 6. Send speeds + sensor request to all robots, then read all replies.
        for (std::size_t i = 0; i < 4; ++i)
            if (alive[i] && !system.sendRobotRequest(i)) alive[i] = false;
        for (std::size_t i = 0; i < 4; ++i) {
            if (alive[i] && !system.receiveRobotSensors(i)) {
                std::cerr << "Robot " << i << " lost. Sending STOP.\n";
                system.stopRobot(i);
                alive[i] = false;
            }
        }

        // 7. Carlos UpdatePheromoneList() every 0.5 s.
        if (halfSecondTick) system.updatePheromones(now);

        ++tick;
        nextTick += period;
        std::this_thread::sleep_until(nextTick);
    }

    std::cout << "STOPPING ALL ROBOTS\n";
    for (std::size_t i = 0; i < 4; ++i) {
        system.stopRobot(i);
        system.disconnectRobot(i);
    }
    std::cout << "Food collected: " << foodCollected
              << ", pheromones left: " << system.pheromoneCount() << "\n";
    return 0;
}
