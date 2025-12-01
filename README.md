## Hi there 👋

<!--
**Med2408-26/Med2408-26** is a ✨ _special_ ✨ repository because its `README.md` (this file) appears on your GitHub profile.

# A* Pathfinding GUI with ROS2 Integration (Tkinter + RViz)

This project implements an **interactive A\* pathfinding visualizer** using **Tkinter** for GUI rendering and **ROS2** for communication with RViz.  
It allows real-time visualization of grid maps, start/goal selection, A\* search progress, and path publishing to RViz.

---

## 🚀 Features

### 🖥️ GUI (Tkinter)
- Draw and erase walls with the mouse  
- Set **start** and **end** positions  
- Visualize:
  - Frontier nodes (open list)
  - Visited cells (closed list)
  - Optimal A* path
- Real-time animation of the A* algorithm  
- Random map generator

### 🤖 ROS2 Integration
- Publishes the grid as an **OccupancyGrid** (`/astar_grid`)
- Publishes the resulting path as a **nav_msgs/Path** (`/astar_path`)
- Subscribes to:
  - `/initialpose` (RViz 2D Pose Estimate → sets start)
  - `/goal_pose` (RViz 2D Goal → sets target)
- Services:
  - `/clear_grid` — Remove all walls  
  - `/reset_planner` — Full reset of A\*, grid, and markers  

### 🎯 A* Algorithm
- Supports **8-direction movement** (with diagonal movement)
- **No corner cutting**
- Octile distance heuristic
- Visualization-friendly generator version of A*

---

## 📦 Requirements

### Python
- Python 3.8+
- Tkinter

### ROS2
- Tested on **ROS2 Jazzy**  
- Works with `rclpy` and standard nav message packages:

```bash
sudo apt install ros-${ROS_DISTRO}-rclpy \
                 ros-${ROS_DISTRO}-nav-msgs \
                 ros-${ROS_DISTRO}-geometry-msgs \
                 ros-${ROS_DISTRO}-std-msgs \
                 ros-${ROS_DISTRO}-std-srvs
