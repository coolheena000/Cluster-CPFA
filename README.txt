CPFA OFFLINE STATE ENGINE - first development milestone

1. Unzip into your existing Cluster-CPFA-physical folder, preserving physical_robot/.
2. From project root run:
   python3 physical_robot/test_cpfa_offline.py
3. This test DOES NOT move the robot or require Wi-Fi/camera.

Reference: source/CPFA/CPFA_controller.cpp from your uploaded source archive.
Implements an approximate, testable subset of CPFA transitions. Survey headings
and Poisson CDF are derived from source. Probabilities and dimensions are DEMO
DEFAULTS, not extracted experiment configuration. Physical pose, nest, food,
pheromone trails and motor navigation are NOT implemented. Do not present
simulated food deliveries as physical experiment results.

Keep existing working run_robot1_search_safe.py untouched.
