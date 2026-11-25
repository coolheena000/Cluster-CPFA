# What I changed, and why

Changes after the 15 October 2026 commit, found while testing on the real
robots. Line numbers point to the working code in `physical_cpp/`.
**(new)** = not part of Carlos's algorithm.

---

## 1. Search: finish each step before choosing the next one
`CPFA_PhysicalCore.h`, line 123

- **Before:** a new 8 cm search target every half second.
- **After:** a new target only when the robot reaches the old one (within 5 cm), or after 4 seconds.
- **Why:** the e-puck moves about 2 cm in half a second, so it kept stopping to turn.
- **Result:** Run 6 → Run 7: forward moves 77 → 106, area covered 10 → 17 cells (15 cm cells; 7 → 12 with 20 cm cells).

```cpp
if (std::hypot(dx, dy) >= 0.05 && ++searchStepAge < 8) return;
```

## 2. Robot always gets the newest command
`EpuckHardware.cpp`, lines 95 and 179

- **Before:** the Mac read one sensor reply per step, and small commands could wait in the network.
- **After:** the Mac reads all waiting replies and keeps the newest one, and commands are sent at once (`TCP_NODELAY`).
- **Why:** a robot kept turning left while the Mac was sending RIGHT, so it was running an old command.
- **Result:** after this change, the robots followed the commands (only 1 or 2 mismatches per run). The original cause isn't proven.

## 3. Food counts only when the robot drives over it **(new value)**
`run_four_robots.cpp`, line 94

- **Before:** `kFoodRadius = 0.13` (Carlos's FoodDistanceTolerance)
- **After:** `kFoodRadius = 0.05`
- **Why:** with 13 cm, a robot near a group of tags picked up tags it never touched, so we couldn't tell which one it found.

## 4. Show which food tag was found
`CPFA_OverheadCamera.h` line 85, `run_four_robots.cpp`

- **Before:** "FOUND FOOD, 15 left", with no tag number.
- **After:** the terminal and the camera window show **"FOOD #1 FOUND: tag 23"**, with a red circle on that tag.
- **Why:** so a screen recording shows which tag to remove.

## 5. Speed
`run_four_robots.cpp`, lines 111 and 112

| | Before | After |
|---|---|---|
| Forward | 300 (3.9 cm/s) | 350 (4.5 cm/s) |
| Turn | 120 (4° per step) | 200 (7° per step) |

- **Why:** the robot spent most of its time turning.
- **Result:** Run 8, 10 minutes: 7 food, area covered 35 cells.

## 6. Robots stop at the end
`run_four_robots.cpp`, line 437

- **Before:** STOP sent once, then the connection closed.
- **After:** STOP sent 5 times, 0.15 s apart, then disconnect.
- **Why:** a robot kept driving after the 10-minute run ended.

## 7. Camera-based robot avoidance **(new)**
`run_four_robots.cpp`, line 50

- **After:** if another robot is within 18 cm in front, the robot does the same turn-away maneuver as for an IR obstacle.
- **Why:** two robots came within about 7.5 cm of each other.
- **Status:** not tested on the robots yet.

## 8. Further from the walls
`run_four_robots.cpp`

- **Before:** `kRobotRadius = 0.06`
- **After:** `kRobotRadius = 0.12`
- **Why:** search targets were too close to the walls and robots hit them.

## 9. Arena size and corner tags
`run_four_robots.cpp`, line 45

- **Before:** 2.54 × 1.68 m
- **After:** temporary square arena, 59 × 59 in (1.50 m), because half of the table broke.
- Corner tags: 10–11 is the long side; I checked that the tags come out square on the camera.

## 10. Food detection with the camera
`run_four_robots.cpp`

- Food tags are read for 3 seconds at the start (not one frame), and detections within 10 cm are merged.
- **Why:** one frame missed tags, and one foam cube showed 2–3 faces, which counted as 2–3 food items.

## 11. LED and sound (display only)
- LED **blinks** while the robot carries food; one sound (Star Wars) when it delivers food at the nest.

## 12. Log file
- Each row now also has the command (LEFT/RIGHT/FORWARD/STOP) and the 8 IR readings, saved every row.
- **Why:** to find out why a robot got stuck. A crashed run had lost its data.

## New test programs
- `test_goto_tag13.cpp`: one robot to the tag 13 corner and back. Worked: reached it in 16.8 s, back at 35.6 s.
- `test_camera_map.cpp`: top-down view to check that the camera covers the arena.
- `test_motor_move.cpp`: time and speed options for the wheel test.

---

**Same as Carlos:** the four CPFA states, the parameter values (0.504, 4.27, 3.75, 0.03 …), the Poisson decisions and pheromone decay.

**Different from Carlos:** search-step timing (Carlos's uninformed search can choose a new step every 0.5 s within about 22 cm of the target), food distance, infrared threshold (20 instead of 0), the response to a target outside the arena, and camera-based movement at 8 ticks a second instead of timed movement at 16.
