diff --git a/README.md b/README.md
index 4d1984a7b4b1f38aa0fe0b262a10783526d7ff3c..f2bee748d22a9769bdcf0a87660ee38c5276a34f 100644
--- a/README.md
+++ b/README.md
@@ -1 +1,140 @@
-# miniproject2
\ No newline at end of file
+# Autonomous Flood-Response Boat (ROS 2 + Gazebo)
+
+This repository is a starter project for building a **small autonomous flood-response boat** (WAM-V style) that can operate without human intervention.
+
+It is designed to support your full path:
+1. **Simulation first** (ROS 2 Humble + Gazebo flood world).
+2. **Autonomy logic development** (waypoint mission + obstacle handling state machine).
+3. **Hardware transition** (Raspberry Pi companion computer + Pixhawk flight controller).
+
+---
+
+## Project goals
+
+- Simulate a flood-affected urban environment in Gazebo.
+- Run a ROS 2 autonomy node that:
+  - accepts waypoints,
+  - navigates waypoint-to-waypoint,
+  - pauses/replans when obstacles are detected.
+- Keep architecture compatible with future real hardware integration via MAVROS/PX4 or ArduPilot.
+
+---
+
+## Repository layout
+
+```text
+.
+├── docs/
+│   └── architecture.md
+├── ros2_ws/
+│   └── src/
+│       └── flood_boat_sim/
+│           ├── flood_boat_sim/
+│           │   ├── __init__.py
+│           │   └── mission_node.py
+│           ├── launch/
+│           │   └── flood_mission.launch.py
+│           ├── params/
+│           │   └── mission.yaml
+│           ├── resource/
+│           │   └── flood_boat_sim
+│           ├── worlds/
+│           │   └── flood_city.world
+│           ├── package.xml
+│           ├── setup.py
+│           └── setup.cfg
+└── README.md
+```
+
+---
+
+## Quick start (simulation)
+
+> Assumes Ubuntu 22.04 with ROS 2 Humble installed.
+
+### 1) Build workspace
+
+```bash
+cd ros2_ws
+colcon build
+source install/setup.bash
+```
+
+### 2) Run mission node + (optional) world path publication
+
+```bash
+ros2 launch flood_boat_sim flood_mission.launch.py
+```
+
+### 3) Run Gazebo with flood world (separate terminal)
+
+```bash
+cd ros2_ws
+source install/setup.bash
+gazebo src/flood_boat_sim/worlds/flood_city.world
+```
+
+> If using `gazebo_ros`, you can replace with a Gazebo ROS launch once your stack is installed.
+
+---
+
+## Current autonomy logic (starter)
+
+`mission_node.py` includes a practical ROS 2 state machine with these states:
+
+- `IDLE`: waiting for waypoints.
+- `NAVIGATING`: moving toward current waypoint.
+- `AVOIDING`: obstacle seen, hold/replan.
+- `COMPLETE`: mission complete.
+
+### Subscribed topics
+
+- `/gps/fix` (`sensor_msgs/NavSatFix`)
+- `/imu/data` (`sensor_msgs/Imu`)
+- `/obstacle_distance_m` (`std_msgs/Float32`)
+- `/mission/waypoints` (`geometry_msgs/PoseArray`)
+
+### Published topics
+
+- `/cmd_vel` (`geometry_msgs/Twist`) — simple simulation command.
+- `/mission/state` (`std_msgs/String`) — current mission state.
+- `/target_waypoint` (`geometry_msgs/PoseStamped`) — active target.
+
+This keeps the simulation simple now while allowing migration to thruster/motor control later.
+
+---
+
+## Hardware transition plan (Raspberry Pi + Pixhawk)
+
+1. Keep mission logic on Raspberry Pi (`mission_node`).
+2. Bridge Pi ↔ Pixhawk with MAVROS or microRTPS (PX4) / MAVLink.
+3. Replace `/cmd_vel` output with:
+   - velocity setpoints, or
+   - attitude/thrust setpoints, or
+   - custom motor mix node.
+4. Add safety layers:
+   - geofence,
+   - return-to-home on heartbeat timeout,
+   - RC/manual override channel.
+
+---
+
+## Integration references you shared
+
+- Intelligent Quads boat setup:
+  - https://github.com/Intelligent-Quads/iq_sim/blob/master/docs/boat_setup.md
+- VRX (Virtual RobotX):
+  - https://github.com/osrf/vrx
+
+Use VRX as your long-term high-fidelity benchmark; use this repo as a focused MVP for flood response autonomy.
+
+---
+
+## Suggested next steps
+
+1. Add a boat URDF/SDF model with buoyancy + dual-thruster plugin.
+2. Replace synthetic obstacle topic with LiDAR/camera perception node.
+3. Add local planner (e.g., DWB/TEB-like behavior for surface vessels).
+4. Add global planner over occupancy grid generated from flood map.
+5. Add mission manager actions (`NavigateToPose`-style behavior).
+
