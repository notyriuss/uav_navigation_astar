# Path Planning and Drone Navigation with A*

## Overview

This repository implements a grid-based algorithm (i.e., **A***) as well as a simple simulation pipeline for navigation. The code can be extended for real-world deployment, once provided with a proper perception, SLAM and control modules.

---


## Requirements

| Dependency     | Minimum Version | Notes |
|----------------|----------------|-------|
| **C++ Compiler** | 11.4.0         | Tested with GCC 11.4 on Ubuntu 22.04. Supports C++17. |
| **CMake**       | 3.22.1         | Used for build configuration. |
| **OpenCV**      | 4.13.0         | Required for visualization |


---

## 1. A* Path Planning

### Advantages
- Guarantees the shortest path in a discretized grid when the environment is fully known.
- Simple to implement and understand.
- Works well for static maps in indoor environments.

### Limitations
- Computational cost increases rapidly with grid size.
- Sensitive to resolution: coarse grids may yield suboptimal paths.
- Does not handle dynamic obstacles or continuous state spaces.
- Requires additional smoothing for realistic robot/drone trajectories.

---

## 2. Evolution to Real Drone Deployment

Deploying on a real drone requires several additional components:

1. **SLAM (Simultaneous Localization and Mapping)**  
   - Provides a **metric map** and estimates the drone's pose in real-time. For vision-based SLAM, ORB-SLAM could be used as an example.

2. **Control Layer**  
   - Converts planned trajectories into moving commands for the robot. A proportional controller may suffice for static scenarios, while MPC could be required for dynamic environments with moving obstacles. MPC could also be used to get a smooth path, if correctly implemented with a penalty function.

Using local descriptors, the robot can localize itself in the map, plan its trajectory (e.g., for exploration or waypoint following), and then execute the plan via the control layer.


---

## 3. Vision Pipeline on Jetson Orin

For a drone equipped with an embedded Jetson Orin, OpenCV is commonly used as the framework for many vision-based SLAM systems. By enabling CUDA support in OpenCV, image preprocessing, feature extraction, and descriptor matching can be performed efficiently on the GPU.

The output of these vision algorithms is typically a set of points representing obstacles and landmarks. To use these points for navigation:

- **Occupancy Map Creation**: Each detected point can be expanded to a small occupied region (e.g., a square or circular area around the point) to account for the robot/drone size and sensor uncertainty.
- **Grid Discretization**: The environment is discretized into a grid (or voxel grid in 3D). Each cell is marked as free or occupied based on the points projected into it.
- **Map Updating**: As new frames are processed, the occupancy map can be updated incrementally, enabling real-time navigation in dynamic or unknown environments.

This processed map can then be fed to planning algorithms (like A*) to generate collision-free trajectories, which are subsequently smoothed and sent to the control layer for execution.


---

## 4. GigE Vision Concepts


- **Jumbo Frames**
  - Ethernet frames with MTU larger than the standard 1500 bytes (typically ~9000 bytes), reducing the number of frames needed to transmit a full image.
  - Lower protocol overhead and CPU load, enabling more stable throughput for large image streams.


- **MTU (Maximum Transmission Unit)**  
  - The largest allowed packet size.  
  - Proper configuration prevents fragmentation and dropped frames.

- **Buffers**  
  - The camera stores captured frames in multiple memory buffers before the host processes them.  
  - Prevents frame loss when the host or network cannot keep up, ensuring the map retains more complete environmental data.


---

## Quick Start

```bash
# Clone repository
git clone https://github.com/notyriuss/path_planning.git
cd path_planning

# Build project
mkdir build && cd build
cmake ..
make

# Run simulation
./path_planning
```

---

## Demonstrations
 

<p float="left">
  <img src="img/1.gif" width="400" />
  <img src="img/2.gif" width="400" />
  <img src="img/3.gif" width="400" />
  <img src="img/4.gif" width="400" />
</p>

---

## TODO

Future work includes implementing full navigation using **ROS2** and simulating the drone in **Gazebo** for more realistic scenarios.  

![TODO](img/todo.png)
