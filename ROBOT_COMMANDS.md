# e-puck2 CPFA: robot commands

All commands used to set up, test and run the physical CPFA on the e-puck2 robots.
Run every command from a terminal on the Mac.

## Robots

| Robot | Wi-Fi name | IP | Robot tag | Status |
|---|---|---|---|---|
| Robot 0 | e-puck2_05367 | 172.20.10.2 | 1 | OK |
| Robot 1 | e-puck2_05840 | 172.20.10.3 | 2 | OK |
| Robot 2 | e-puck2_05333 | 172.20.10.4 | 3 | sensor data invalid, needs main firmware (or replace with the new robot) |
| Robot 3 | e-puck2_05359 | 172.20.10.5 | 4 | OK |
| Spare | e-puck2_05297 | — | — | unknown Wi-Fi password |

Corner tags: 10 (top-left), 11 (top-right), 12 (bottom-right), 13 (bottom-left).
Food cubes: tag 0. All tags are AprilTag tag36h11.

## Before every session

1. Phone hotspot **RobotHotSpot** on, "Maximize Compatibility" on.
2. Mac connected to **RobotHotSpot** (IP must start with `172.20.10.`):
   ```bash
   ifconfig en0 | grep "inet "
   ```
3. Each robot: selector on **F (15)**, then switch off and on, wait 20 s.
4. Only one program talks to a robot at a time. Do not run `nc`.

Go to the code folder:
```bash
cd /Users/henaparween/workplace/Carlos_hena_work/Cluster-CPFA-Physical-CPP/physical_cpp
```

## 1. Network and communication checks

Find the robots on the hotspot (prints IP and MAC address):
```bash
for i in $(seq 1 13); do ping -c 1 -t 1 172.20.10.$i >/dev/null 2>&1 & done; wait; arp -an | grep 172.20.10
```

One robot, sensor read:
```bash
clang++ -std=c++17 test_sensors.cpp EpuckHardware.cpp -o test_sensors
./test_sensors
```

All robots at once (motors stay at zero):
```bash
clang++ -std=c++17 test_four_connections.cpp EpuckHardware.cpp -o test_four_connections
./test_four_connections 172.20.10.2 172.20.10.3 172.20.10.4 172.20.10.5
```
Expected: `4 of 4 robots OK`, proximity values between 0 and about 100 when nothing is close.

## 2. Motor tests

One robot: forward, left, right, back, each 1 s, with STOP in between:
```bash
clang++ -std=c++17 test_motor_move.cpp EpuckHardware.cpp -o test_motor_move
./test_motor_move 172.20.10.2
```

Obstacle + robot camera photo (one robot):
```bash
clang++ -std=c++17 test_explore_photo.cpp EpuckHardware.cpp -o test_explore_photo
./test_explore_photo 172.20.10.2 60 3
open obstacle_1.bmp
```

Robot camera photo only:
```bash
clang++ -std=c++17 test_camera.cpp -o test_camera
./test_camera 172.20.10.2 epuck_photo.bmp
open epuck_photo.bmp
```

## 3. Arena test without overhead camera

All robots drive together and avoid walls and each other with Carlos's collision maneuver.
First number = seconds, then robot IPs:
```bash
clang++ -std=c++17 test_arena_explore.cpp EpuckHardware.cpp -o test_arena_explore
./test_arena_explore 60 172.20.10.2 172.20.10.3 172.20.10.5
```
Stop with Ctrl+C. All robots get STOP.

## 4. Overhead camera check (robots not used)

```bash
export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
clang++ -std=c++17 test_camera_view.cpp $(pkg-config --cflags --libs opencv5) -o test_camera_view
./test_camera_view        # camera 0 (laptop)
./test_camera_view 1      # camera 1 (e.g. iPhone Continuity Camera)
```
Move the camera until the window shows **CALIBRATED** (all four corner tags visible). Press Q to quit.

## 5. Full CPFA run, 4 robots

Set the arena size, camera index and robot IPs at the top of `run_four_robots.cpp` first.
```bash
export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
clang++ -std=c++17 run_four_robots.cpp EpuckHardware.cpp $(pkg-config --cflags --libs opencv5) -o run_four_robots
./run_four_robots
```
Expected: `Camera calibrated` → `FOOD from camera (N cubes)` → robots connect → states printed every second → `FOUND FOOD` / `DELIVERED FOOD`.
Stop with Q in the camera window or Ctrl+C.

## 6. Offline tests (no robots needed)

```bash
for t in test_collision test_surveying test_controller_integration test_physical_integration test_four_robots test_collision_maneuver; do
  clang++ -std=c++17 $t.cpp EpuckHardware.cpp -o $t && ./$t | tail -1
done
export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
clang++ -std=c++17 test_overhead_camera.cpp $(pkg-config --cflags --libs opencv5) -o test_overhead_camera && ./test_overhead_camera
```

## 7. Setting up a new robot

Do one robot at a time, connected by USB and switched on.

Wi-Fi radio firmware (needed if the robot only shows up in Bluetooth):
```bash
cd /Users/henaparween/workplace/Carlos_hena_work/Cluster-CPFA-Physical-CPP/epuck2_wifi_flash
./flash_slow.sh
```
Expected: `Hash of data verified` three times.

Main processor factory firmware (needed if sensor values look random):
```bash
cd /Users/henaparween/workplace/Carlos_hena_work/Cluster-CPFA-Physical-CPP/epuck2_main_flash
./flash_main.sh
```
Expected: `.data` in the output.

Join the robot to the hotspot:
1. Unplug USB, selector F, switch off and on (LED2 red).
2. Connect the Mac to Wi-Fi `e-puck2_0XXXX`, password `e-puck2robot`.
3. Send the hotspot name and password (replace `<hotspot password>`):
   ```bash
   curl -m 30 -X POST http://192.168.1.1/connect.json -H 'X-Custom-ssid: RobotHotSpot' -H 'X-Custom-pwd: <hotspot password>'
   curl -m 30 http://192.168.1.1/status.json
   ```
   `"urc":0` means connected.
4. Switch the robot off and on (LED2 green). Connect the Mac back to RobotHotSpot.

Erase a robot's saved Wi-Fi: hold the USER button 2 s (LED2 red), then switch off and on.

## LED2 meaning

| LED2 | Meaning |
|---|---|
| Red | Robot's own Wi-Fi (setup mode) |
| Green | Connected to the hotspot |
| Off | Saved network not found |
| Blue blinking | Sending a camera image |

## Troubleshooting

| Problem | Fix |
|---|---|
| `Operation timed out` | Mac not on RobotHotSpot, or robot still booting. Wait 20 s and retry. |
| `CONNECTED but no sensor data` | Selector not on F. Set F and switch the robot off and on. |
| Random sensor values (e.g. 63116) | Flash main processor firmware (`flash_main.sh`). |
| `Camera NOT calibrated` | Corner tags 10–13 not all visible. Use `test_camera_view`. |
| `Failed to connect to 192.168.1.1` | Mac is not on the robot's `e-puck2_0XXXX` Wi-Fi. |
