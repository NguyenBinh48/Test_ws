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

---

## ⚙️ PID Control Info
- The **PID control** is inside the `listener_callback` function of the `follow_aruco` node.
- It is currently a **simple implementation** using a **low-pass filter** for smoothing.

---

## 🤖 Running on Real Robot
- Instead of running `follow_aruco`, use:
```bash
ros2 run aruco_tracker follow_aruco_irl
```

---

## 🧪 Dependencies

- ✅ Make sure your **OpenCV version is > 4.10**  
Check with:
```bash
python3 -c "import cv2; print(cv2.__version__)"
```

- ✅ Check your **Gazebo version**:
```bash
gazebo --version
```
Expected output:
```
Gazebo multi-robot simulator, version 11.10.2
```

---

✅ Ready to simulate or drive your robot in real life!
