# 🤖 Robot ArUco Tracker Simulation

## 🚀 Quick Start

- **Run Gazebo Simulation**  
```bash
ros2 launch robot launch_sim.launch.py
```

- **Launch ArUco Tracker (detect + follow nodes)**  
```bash
ros2 launch aruco_tracker aruco_main.launch.py
```

- **Run detect_aruco node only** 🧩  
```bash
ros2 run aruco_tracker detect_aruco
```
*(Put an ArUco marker in front to check `/aruco_position` topic.)*

- **Run follow_aruco node only** 🎯  
```bash
ros2 run aruco_tracker follow_aruco
```

- **Debug/See camera view** 🎥  
```bash
ros2 run aruco_tracker debug_aruco
```
