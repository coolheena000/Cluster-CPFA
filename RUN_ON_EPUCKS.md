# Running the CPFA on the e-puck2 robots

The code that has run on the robots is in `physical_cpp/`. It needs macOS.
`physical_port/` is a newer version of the decision code, tested only offline.

**Read first:** food is located by the overhead camera, not by a robot sensor.
CPFA has been run with one and two robots. Start with one.

## 1. What you need

- Mac with Xcode command line tools, Homebrew and OpenCV 5 (`brew install opencv pkg-config`)
- USB overhead camera that sees the whole arena. The code uses camera 0; with the USB
  camera unplugged, camera 0 can be the MacBook's own camera
- NETGEAR15 router (ask for the password; it is not in this repository)
- e-puck2 robots already set up for NETGEAR15, selector at 15 (F)
- AprilTags (tag36h11), printed from `tags_to_print/`:
  - corners, flat on the floor: 10 top-left, 11 top-right, 12 bottom-right, 13 bottom-left, as the camera sees them
  - robots, on top: 05367 = tag 1, 05333 = tag 2, 05359 = tag 3, 05417 = tag 4
  - food, flat on the floor: 16 tags, IDs 20 to 35

## 2. Arena size

The code is set for 59 x 59 in (1.50 m) between the corner tag centres.
For another size, change `kArenaHalfX` and `kArenaHalfY` (half the distance, in metres)
in `physical_cpp/run_four_robots.cpp`. A wrong size gives wrong positions.

## 3. Get and build

```bash
git clone -b physical-epuck4 https://github.com/coolheena000/Cluster-CPFA.git
cd Cluster-CPFA
cp physical_cpfa_code/5_main_program/run_cpfa.sh physical_cpp/
cd physical_cpp
export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
clang++ -std=c++17 run_four_robots.cpp EpuckHardware.cpp $(pkg-config --cflags --libs opencv5) -o run_four_robots
```

## 4. Run

1. Switch on the robot and wait about 20 s.
2. Join **NETGEAR15** from the Wi-Fi menu before running the script.
3. Put the robot near the centre (nest) with its tag visible.
4. One robot: `./run_cpfa.sh 05367`. Two robots: `./run_cpfa.sh 05367 05359`.
5. If a robot is not found, read its IP from the router's Attached Devices page and run
   `./run_four_robots IP0 IP1 IP2 IP3` with `-` for robots not used, for example
   `./run_four_robots 192.168.1.2 - - -`.

Stop with Ctrl+C, or Q in the camera window. Keep a hand near the robot's power switch.
A run lasts at most 10 minutes.

## 5. What you should see

- "Camera calibrated", a list of food tags, then one status line per robot per second
- states DEPARTING, SEARCHING, SURVEYING, RETURNING
- "FOUND FOOD tag N" when the robot is within 5 cm of a tag (remove that tag), and a blinking LED
- "DELIVERED FOOD" and a short sound at the nest (centre, radius 0.25 m)
- `cpfa_run_log.csv`; rename it after each run, because the next run overwrites it

## 6. Offline check (no robot)

```bash
cd physical_port/test
./extract_carlos.sh
clang++ -std=c++17 -O2 test_same_as_carlos.cpp ../CPFA_controller.cpp ../CPFA_nest.cpp ../Pheromone.cpp -o test_same_as_carlos
./test_same_as_carlos
```

## Known limits

- Camera-based avoidance between two robots (18 cm) has not been tested on robots.
- In one run the Wi-Fi link was lost after about 251 s; the program does not reconnect.
- The behavior is not identical to the simulation; see `physical_cpfa_code/README.md`.
