# Robot ArUco Tracker Simulation

## Run Gazebo Simulation
```bash
ros2 launch robot launch_sim.launch.py
```

## Launch ArUco Tracker (both detect_aruco and follow_aruco nodes)
```bash
ros2 launch aruco_tracker aruco_main.launch.py
```

## Run detect_aruco node only
```bash
ros2 run aruco_tracker detect_aruco
```
*(Place an ArUco marker in front to check if `/aruco_position` topic works.)*

## Run follow_aruco node only
```bash
ros2 run aruco_tracker follow_aruco
```

## Debug camera view
```bash
ros2 run aruco_tracker debug_aruco
```
