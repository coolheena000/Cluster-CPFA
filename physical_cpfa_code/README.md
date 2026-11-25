# Physical CPFA on e-puck2: code overview

This folder is a copy of the code that runs Carlos's baseline CPFA on real
e-puck2 robots, sorted by what each part does. It's for reading and review.
Robot runs use the working copy in `physical_cpp/`, where these files came from
on 17 November 2026.

## How the system fits together

```
Overhead camera ──> MacBook (C++) ──Wi-Fi──> e-puck2 robots
 (AprilTags:          CPFA decisions,          drive the wheels,
  robot and food      navigation, collision    send back 8 IR readings
  positions)          avoidance, food check
```

Eight times a second the Mac reads the camera, runs one CPFA step for each
robot, chooses LEFT, RIGHT, FORWARD or STOP, sends the wheel speeds, and reads
the robot's IR sensors.

## Folders

| Folder | What it does | Carlos's ARGoS version |
|---|---|---|
| `1_cpfa_algorithm/` | The four CPFA states (departing, searching, surveying, returning), parameters, site fidelity, pheromones and decay | `CPFA_controller.cpp`, `Pheromone.cpp` |
| `2_movement_and_collision/` | Turning and driving to a target; IR collision vector; turn away then 8 cm forward | `BaseController::Move()`, `CollisionDetection()` |
| `3_robot_communication/` | Wi-Fi link to each e-puck2 (21-byte command, 105-byte sensor reply) and per-robot state | ARGoS sensors and actuators |
| `4_camera/` | Overhead camera: corner tags 10-13 calibrate the arena, robot tags 1-4 give position and heading, other tags are food | ARGoS positioning sensor |
| `5_main_program/` | `run_four_robots.cpp`: main loop, food list, nest, logging. `run_cpfa.sh` finds the robots on the router and starts it | `CPFA_loop_functions` |
| `6_tests/` | Offline tests, a single-robot drive to the tag 13 corner and back, a motor test, and a camera coverage map | none |
| `7_results/` | Run logs (one row per robot per second) | none |

## Main files to read first

1. `1_cpfa_algorithm/CPFA_PhysicalCore.h`: the CPFA logic
2. `5_main_program/run_four_robots.cpp`: how one control step works
3. `2_movement_and_collision/CPFA_CollisionManeuver.h`: Carlos's collision maneuver

## Where the physical version differs from Carlos's simulation

These changes were needed for real robots. The four states and the
parameter values are the same; the behavior below is not.

| Change | Why |
|---|---|
| Position comes from an overhead camera and AprilTags | Real robots have no positioning sensor |
| Control loop runs 8 times a second (ARGoS: 16) | Camera and Wi-Fi rate |
| A robot stops if the camera loses it for 0.5 s | Safety |
| Food counts when the robot is within 5 cm of a food tag (Carlos: 0.13 m) | So the robot has to drive over a tag, and it's clear which tag was found |
| A new 8 cm search step is chosen only after the robot reaches the previous one, or after 4 s | The e-puck moves about 2 cm per half second, so a new step every half second made it turn in place |
| Camera-based avoidance of other robots within 18 cm in front | Safety; not part of Carlos's algorithm |
| Forward speed 350, turn speed 200 (e-puck steps per second) | e-puck2 is much slower than the ARGoS foot-bot (16 cm/s) |
| Infrared readings below 20 are ignored (Carlos: 0) | Sensor noise |
| A target outside the arena becomes an 8 cm step at 37.5 degrees (Carlos: current position and a 37.5 degree right turn) | Not matched yet |
| LED blinks while carrying food; one sound on delivery | Display only, to see the behaviour |

## Current test settings in `run_four_robots.cpp`

Temporary square arena 59 x 59 in (half of the table is broken), 16 food tags,
10-minute runs. The full arena values are kept in the comments.

## Results so far

| Run | Robots | Food | Time | Food delivered |
|---|---|---|---|---|
| run05 | 2 | 16 cubes | 600 s | 10 |
| run06 | 1 | 16 tags | 300 s | 3 |
| run07 (search fix) | 1 | 16 tags | 300 s | 3; more area covered (12 cells of 20 cm, run06: 7) |
| run08 (faster, 13 cm food distance) | 1 | 16 tags | 600 s | 7 |
| run09 (5 cm food distance; log overwritten, result from the video) | 1 | 16 tags | 601 s | 5 |
| run10 (Wi-Fi lost) | 1 | 12 tags | about 251 s active | 1 |

These are test runs. The setup changed between them, so they are not repeated
trials of one experiment.

## Building (from the repository root)

```bash
export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
D=physical_cpfa_code
clang++ -std=c++17 -I$D/1_cpfa_algorithm -I$D/2_movement_and_collision \
    -I$D/3_robot_communication -I$D/4_camera \
    $D/5_main_program/run_four_robots.cpp $D/3_robot_communication/EpuckHardware.cpp \
    $(pkg-config --cflags --libs opencv5) -o run_four_robots
```

The scripts in this folder have the router password removed. Use the ones in
`physical_cpp/` to run the robots.
