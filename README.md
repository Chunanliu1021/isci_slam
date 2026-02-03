# HDL Graph SLAM & ISCI Robot Workspace

This repository contains the configuration and launch files for running HDL Graph SLAM in both simulation and real-world environments with the ISCI robot.

## 1. Simulation Environment

### Launch SLAM Simulation
Start the robot model and simulation environment:
```bash
roslaunch isci_robot iscigz_slam.launch
Note: It is normal to see TF tree errors for the robot model initially. The model's TF tree is linked to the map frame, which will resolve correctly once the SLAM launch file is active.

Start HDL Graph SLAM
Run the SLAM node configured for the simulation:

Bash

roslaunch hdl_graph_slam hdl_graph_slam_isci_sim.launch
2. Robot Control (Keyboard)
To control the mobile manipulator's movement, use the following command:

Bash

rosrun isci_robot pubWheelSpeedq
Control Mapping: | Key | Action | | :--- | :--- | | W / S | Forward / Backward | | A / D | Left / Right | | X | Stop | | Q | Exit (Note: Ctrl+C is disabled due to getch implementation) |

3. Map Management
Saving the Map
Use the rosservice to export the generated point cloud. Use the Tab key to auto-complete the service structure:

Bash

rosservice call /hdl_graph_slam/save_map "utm: false
resolution: 0.05
destination: '/home/leo/plan_ws/src/hdl_graph_slam/map/map.pcd'"
Visualizing the Map
Modify the file path in visualize.py to match your saved .pcd file.

Execute the visualization script:

Bash

cd ~/plan_ws/src/hdl_graph_slam/scripts
python3 visualize.py
4. Real-World Hardware Testing
Step 1: Initialize Velodyne VLP-16
Bash

roslaunch velodyne_pointcloud VLP16_points.launch
Step 2: Initialize Robot Control
Bash

roslaunch isci_robot final_real.launch
Step 3: Launch SLAM for Mapping
Bash

roslaunch hdl_graph_slam hdl_graph_slam_isci_real.launch
Step 4: Save Real-World Map
Bash

rosservice call /hdl_graph_slam/save_map "utm: false
resolution: 0.05
destination: '/home/tm5/caliu_workspace/ocs2_ws/src/hdl_graph_slam/map/map632.pcd'"
