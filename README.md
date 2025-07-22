# Lane_following_opencv_ROS2

# 🏎️ ROS2 Line Follower 

This package implements a **line-following robot** using **ROS2**, **OpenCV**, and **LaserScan**.  
The robot follows **yellow and white lane lines** and switches to obstacle avoidance mode if an obstacle is detected.

---

## ✅ Features
- Follows **yellow and white lane lines** using OpenCV (HSV masking)  
- Switches between **line following** and **turning mode** if one line disappears  
- **Obstacle avoidance** using LaserScan (`/scan` topic)  
- Uses **MultiThreadedExecutor** for concurrent camera and LiDAR callbacks  
- Adjustable **linear & angular velocity gains** for tuning

---

## 🛠 Requirements
- **ROS 2 (Humble or later)**  
- **Python 3.8+**  
- Packages:
  - `rclpy`
  - `sensor_msgs`
  - `geometry_msgs`
  - `cv_bridge`
  - `opencv-python`
  - `numpy`

Install missing Python dependencies:

```bash
pip install opencv-python numpy
cd ~/ros2_ws/src
git clone <your_repo_url> line_follower
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run line_follower line_follower
self.linear_velocity_gain = 0.1      # Forward speed
self.slow_linear_velocity_gain = 0.05
self.angular_velocity_gain = 0.04    # Normal turns
self.fast_angular_velocity_gain = 0.04  # Sharp turns
