CPFA robot 1: bounded physical SEARCHING stage only, not a complete Carlos CPFA port.

Extract in ~/epuck_project/Cluster-CPFA-physical:
  unzip -n ~/Downloads/CPFA_robot1_physical_search.zip

With robot OFF, check no movement (default):
  python3 physical_robot/run_cpfa_robot1_physical.py

Only after verifying the dry run and with wheels raised, robot powered on and Wi-Fi connected:
  python3 physical_robot/run_cpfa_robot1_physical.py --duration 3 --speed 30 --enable-motors

Emergency STOP from another terminal (if network still works):
  python3 physical_robot/run_cpfa_robot1_physical.py --stop-only

If STOP is ineffective, immediately use robot physical power switch.
Physical localization, food sensing, site fidelity and pheromone mechanisms are NOT implemented here.
