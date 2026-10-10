CPFA integrated offline loop

1. Extract this ZIP inside ~/epuck_project/Cluster-CPFA-physical:
   unzip -n ~/Downloads/CPFA_full_loop_offline.zip
2. Run:
   python3 physical_robot/test_cpfa_full_loop_offline.py

Requires previously extracted physical_robot/cpfa_state_engine.py and
physical_robot/cpfa_navigation.py. It uses no hardware or network.
It does not replace any existing project files.
