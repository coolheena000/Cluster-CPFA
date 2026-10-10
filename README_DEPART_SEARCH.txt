CPFA DEPARTING + SEARCHING physical prototype
===========================================
Extract inside ~/epuck_project/Cluster-CPFA-physical
Existing run_robot1_search_safe.py is required and unchanged.

1. Robot OFF; run offline:
python3 physical_robot/run_cpfa_depart_search.py

2. Only if dry-run works, lift robot wheels off floor, keep power switch accessible,
connect to robot hotspot, and run:
python3 physical_robot/run_cpfa_depart_search.py --duration 3 --depart-seconds 1 --speed 30 --enable-motors

3. Check wheels stopped. If not, physically switch OFF immediately.

This is a timed, bounded physical state demo, NOT Carlos CPFA equivalence.
A camera/localization and actual food/nest perception are needed for true
position-driven CPFA state transitions.
