CPFA e-puck2: OFFLINE waypoint navigation test

Extract in ~/epuck_project/Cluster-CPFA-physical:
  unzip -n ~/Downloads/CPFA_navigation_offline.zip

Run:
  python3 physical_robot/test_cpfa_navigation_offline.py

This is a standalone differential-drive navigation component and test.
It DOES NOT control real e-puck2 hardware or yet call CPFAEngine.step().
The robot pose used here is simulated. For physical waypoint navigation,
a measured pose from overhead tracking or validated localization is needed.
Wheel dimensions are approximate and must be calibrated before physical use.
Existing safe Wi-Fi controller files are untouched.
