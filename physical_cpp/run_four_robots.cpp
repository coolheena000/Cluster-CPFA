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
// Run:   ./run_cpfa.sh 05367 05333   (finds the robots by MAC, then runs this)
//        ./run_four_robots IP0 IP1 IP2 IP3   ("-" = robot not used)
// Stop:  Ctrl+C or Q in the camera window (all robots get a STOP command)
// Log:   cpfa_run_log.csv (one row per robot per second)

#include "CPFA_CollisionManeuver.h"
#include "CPFA_FourRobotSystem.h"
#include "CPFA_OverheadCamera.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <thread>

// ===================== EDIT FOR YOUR ARENA =====================
// Robot i carries AprilTag tag36h11 ID i+1 (1..4); corners 10..13; food cubes ID 0.
// Robot IPs come from the command line (run_cpfa.sh finds them by MAC):
//   robot 0 = e-puck2_05367 (tag 1), robot 1 = e-puck2_05333 (tag 2),
//   robot 2 = e-puck2_05359 (tag 3), robot 3 = e-puck2_05417 (tag 4)
static const int kRobotPort = 1000;

// Arena is centred on (0,0), metres. x to the right, y up, heading CCW.
// Lab arena: these are the distances between corner tag centres (10-11 long side),
// about 2.12 m x 1.37 m. Inside the wooden frame it is 91 in x 55.5 in (2.31 m x 1.41 m).
// Temporary square arena (half of the table is broken): 59 in x 59 in (1.50 m)
// between the corner tags. Full arena values were 1.06 and 0.685.
static const double kArenaHalfX = 0.749;
static const double kArenaHalfY = 0.749;
static const double kRobotRadius = 0.12;  // e-puck2 radius 0.035 + wall margin
// The camera sees both robots, so a robot ahead closer than this (centre to
// centre) starts the same collision maneuver as a front IR obstacle.
static const double kRobotAvoidDistance = 0.18;

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
static std::vector<int> gFoodTag;  // AprilTag ID of each food item (-1 = virtual)
static std::vector<cv::Point2f> gFoodPx;  // where each food tag is in the camera image
// Food found so far, drawn on the camera window so a screen recording shows
// which tag to remove.
struct FoundFood { cv::Point2f px; int tag; int number; double time; };
static std::vector<FoundFood> gFound;

static cv::Mat markFoundFood(const cv::Mat& frame, double now) {
    cv::Mat view = frame.clone();
    for (const auto& f : gFound) {
        cv::circle(view, f.px, 45, cv::Scalar(0, 0, 255), 5);
        cv::putText(view, "#" + std::to_string(f.number) + " tag " + std::to_string(f.tag),
                    f.px + cv::Point2f(50, -20), cv::FONT_HERSHEY_SIMPLEX, 1.2,
                    cv::Scalar(0, 0, 255), 3);
    }
    if (!gFound.empty() && now - gFound.back().time < 8.0) {
        const auto& f = gFound.back();
        const std::string text = "FOOD #" + std::to_string(f.number) + " FOUND: tag " +
                                 std::to_string(f.tag) + " - remove it";
        cv::rectangle(view, {0, 0}, {view.cols, 90}, cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(view, text, {20, 65}, cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 5);
    }
    return view;
}

// Carlos FoodDistanceTolerance = 0.13 m. On the real arena the robot must drive
// over the food tag (robot centre within 5 cm of the tag centre), so it is clear
// which tag was found.
static const double kFoodRadius = 0.05;
// Food tags on the floor: how many should be there, and how long to look for them.
static const int kExpectedFood = 16;
static const double kFoodScanSeconds = 3.0;
// Detections closer than this are one food item (a cube shows 2-3 faces at once).
static const double kFoodMergeRadius = 0.10;
// Sound when a robot finds food (e-puck2 speaker bits, see EpuckHardware.h).
// Display only: a short jingle = the start of a firmware melody, cut off by
// sending "stop" (0x20) after a few seconds so four robots don't get noisy.
// Melodies: 0x01 Mario, 0x02 Underworld, 0x04 Star Wars, 0x40 We Wish You;
// beeps: 0x08 short, 0x10 long.
static const uint8_t kFoodDeliveredSound = 0x04;    // Star Wars, at the nest
static const double kFoodDeliveredSeconds = 3.0;
static const uint8_t kStopSound = 0x20;
// Display only: the green body LED blinks while a robot carries food.

static const int kTicksPerSecond = 8;
static const int16_t kForwardSpeed = 350;  // steps/s, 1000 = 12.9 cm/s (was 300)
static const int16_t kTurnSpeed = 200;     // was 120; about 7 degrees per control tick
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

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::array<std::string, 4> robotIps{};
    std::array<bool, 4> used{};
    for (std::size_t i = 0; i < 4; ++i) {
        if (static_cast<int>(i) + 1 < argc && std::string(argv[i + 1]) != "-") {
            robotIps[i] = argv[i + 1];
            used[i] = true;
        }
    }
    if (std::find(used.begin(), used.end(), true) == used.end()) {
        std::cerr << "Usage: ./run_four_robots IP0 IP1 IP2 IP3   (\"-\" = not used)\n"
                     "Easier: ./run_cpfa.sh 05367 05333\n";
        return 1;
    }

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
    CPFAFourRobotSystem system(robotIps);
    for (std::size_t i = 0; i < 4; ++i) {
        // EpuckHardware takes the port in its constructor; default is 1000.
        (void)kRobotPort;
        if (!used[i]) continue;
        if (!system.connectRobot(i)) {
            std::cerr << "Robot " << i << " (" << robotIps[i] << ") not connected. Stopping all.\n";
            for (std::size_t j = 0; j < i; ++j) { system.stopRobot(j); system.disconnectRobot(j); }
            return 1;
        }
        std::cout << "Robot " << i << " connected (" << robotIps[i] << ", tag " << i + 1 << ")\n";
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

    // Food = food tags the overhead camera sees (any tag that is not a
    // robot or corner tag, e.g. IDs 20-35 flat on the floor). One frame can
    // miss a tag, so collect over a few seconds and merge repeats of the same
    // tag or cube (same spot within kFoodMergeRadius). Without food tags, fall back to Carlos's
    // ClusterFoodDistribution() with fixed cluster centres.
    std::cout << "Reading food tags (" << kFoodScanSeconds << " s)...\n";
    const auto scanStart = std::chrono::steady_clock::now();
    while (!gStop && std::chrono::duration<double>(
               std::chrono::steady_clock::now() - scanStart).count() < kFoodScanSeconds) {
        overhead.update();
        for (const auto& f : overhead.foodSeen()) {
            const bool known = std::any_of(gFood.begin(), gFood.end(), [&](const CPFATarget& g) {
                return std::hypot(g.x - f.x, g.y - f.y) < kFoodMergeRadius;
            });
            if (!known) { gFood.push_back({f.x, f.y}); gFoodTag.push_back(f.id); gFoodPx.push_back({f.px, f.py}); }
        }
        if (kShowCameraWindow) {
            cv::imshow("CPFA overhead camera", overhead.debugFrame());
            cv::waitKey(1);
        }
    }
    if (!gFood.empty()) {
        std::cout << "FOOD from camera (" << gFood.size() << " tags):\n";
        if (static_cast<int>(gFood.size()) != kExpectedFood)
            std::cout << "WARNING: expected " << kExpectedFood << " food tags, saw "
                      << gFood.size() << ". Check that every food tag is flat and visible.\n";
    } else {
        for (const auto& c : kClusterCenters) {
            const double x0 = c.x - (kClusterWidthX - 1) * kFoodOffset / 2.0;
            const double y0 = c.y - (kClusterLengthY - 1) * kFoodOffset / 2.0;
            for (int j = 0; j < kClusterLengthY; ++j)
                for (int k = 0; k < kClusterWidthX; ++k)
                    { gFood.push_back({x0 + k * kFoodOffset, y0 + j * kFoodOffset}); gFoodTag.push_back(-1); gFoodPx.push_back({-100, -100}); }
        }
        std::cout << "No food cubes seen. Virtual FOOD (" << gFood.size() << " items):\n";
    }
    for (std::size_t f = 0; f < gFood.size(); ++f)
        std::printf("  tag %d at (%.2f, %.2f)\n", gFoodTag[f], gFood[f].x, gFood[f].y);

    std::array<CameraPose, 4> camera{};
    std::array<double, 4> soundStopAt{};  // 0 = no jingle playing
    std::array<CPFACollisionManeuver, 4> maneuvers = {{
        {kForwardSpeed, kTurnSpeed}, {kForwardSpeed, kTurnSpeed},
        {kForwardSpeed, kTurnSpeed}, {kForwardSpeed, kTurnSpeed}}};
    std::array<bool, 4> alive = used;
    bool motorsLockedWarned = false;
    unsigned int foodCollected = 0;
    const std::size_t foodAtStart = gFood.size();

    std::ofstream log("cpfa_run_log.csv");
    log << "time_s,robot,state,x_m,y_m,heading_rad,pose_valid,holding_food,food_collected_total,"
           "move,ir0,ir1,ir2,ir3,ir4,ir5,ir6,ir7\n";

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
            cv::imshow("CPFA overhead camera", markFoundFood(overhead.debugFrame(), now));
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

            if (soundStopAt[i] > 0.0 && now >= soundStopAt[i]) {
                system.playRobotSound(i, kStopSound);
                soundStopAt[i] = 0.0;
            }

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
                        std::printf("Robot %zu FOUND FOOD tag %d at (%.2f, %.2f), %zu left. Remove this tag.\n",
                                    i, gFoodTag[f], gFood[f].x, gFood[f].y, gFood.size() - 1);
                        gFound.push_back({gFoodPx[f], gFoodTag[f], static_cast<int>(gFound.size()) + 1, now});
                        gFood.erase(gFood.begin() + static_cast<long>(f));
                        gFoodTag.erase(gFoodTag.begin() + static_cast<long>(f));
                        gFoodPx.erase(gFoodPx.begin() + static_cast<long>(f));
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
                system.playRobotSound(i, kFoodDeliveredSound);
                soundStopAt[i] = now + kFoodDeliveredSeconds;
            }
            // Display only: the body LED blinks while the robot carries food
            // (no sound); the delivery jingle at the nest is the only sound.
            system.setRobotBodyLed(i, core.isHoldingFood && (tick / 2) % 2 == 0);

            // 5. Collision avoidance (Carlos's maneuver) + navigation -> wheel speeds.
            CPFAMovement movement;
            int16_t speed;
            auto collision = CPFAPhysicalCollision{}.calculate(
                system.getRobotSensors(i), kEpuckSensorAngles, true);
            for (std::size_t j = 0; j < 4 && pose.valid && !collision.obstacleDetected; ++j) {
                if (j == i || !alive[j] || now - camera[j].receivedAt > kPoseTimeout) continue;
                const double dx = camera[j].x - pose.x, dy = camera[j].y - pose.y;
                if (std::hypot(dx, dy) > kRobotAvoidDistance) continue;
                const double bearing = std::atan2(dy, dx) - pose.heading;
                const double degrees = std::atan2(std::sin(bearing), std::cos(bearing)) * 180.0 / 3.141592653589793;
                if (std::fabs(degrees) <= 37.5) {
                    collision.obstacleDetected = true;
                    collision.collisionAngle = degrees;
                }
            }
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
                log << now << ',' << i << ',' << stateName(core.state) << ','
                    << pose.x << ',' << pose.y << ',' << pose.heading << ','
                    << pose.valid << ',' << core.isHoldingFood << ',' << foodCollected << ','
                    << moveName(movement);
                for (auto v : system.getRobotSensors(i).proximity) log << ',' << v;
                log << '\n' << std::flush;  // keep the data even if the run is killed
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

    // Wheels keep the last speed after the socket closes, so send STOP several
    // times and give the robot time to act on it before disconnecting.
    std::cout << "STOPPING ALL ROBOTS\n";
    for (int repeat = 0; repeat < 5; ++repeat) {
        for (std::size_t i = 0; i < 4; ++i)
            if (alive[i]) system.stopRobot(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    for (std::size_t i = 0; i < 4; ++i)
        if (used[i]) system.disconnectRobot(i);
    const double runSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::printf("Run time: %.0f s\n", runSeconds);
    std::cout << "Food collected: " << foodCollected << " of " << foodAtStart
              << ", pheromones left: " << system.pheromoneCount() << "\n";
    return 0;
}
