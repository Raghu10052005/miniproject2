# System Architecture (Simulation → Real Boat)

## 1) High-level data flow

```text
[Mission Planner Node] ---> [/cmd_vel or setpoints] ---> [Pixhawk / Motor Mixer] ---> [Thrusters]
        ^                          |
        |                          v
 [Perception + Mapping] <--- [Sensors: GPS/IMU/LiDAR/Camera]
```

In simulation, `/cmd_vel` goes to Gazebo plugins.
On hardware, command outputs should be routed through Pixhawk-compatible interfaces.

## 2) ROS 2 graph (MVP)

- `flood_boat_sim/mission_node`
  - Inputs: GPS, IMU, obstacle distance, mission waypoints
  - Outputs: cmd_vel, mission state, target waypoint

## 3) Frame conventions

- `map`: global frame for waypoint mission.
- `base_link`: boat body frame.
- `gps_link`, `imu_link`, `lidar_link`: sensor frames.

Use TF2 static transforms for rigidly mounted sensors.

## 4) State machine behavior

- `IDLE`
  - Wait until waypoint list is received.
- `NAVIGATING`
  - Track heading and distance to target waypoint.
  - Publish forward speed + yaw command.
- `AVOIDING`
  - Triggered when obstacle range < threshold.
  - Stop forward speed and yaw slowly to search a free sector.
- `COMPLETE`
  - Stop when final waypoint reached.

## 5) Hardware safety requirements

- Manual override channel from RC controller.
- Geofence in both mission node and autopilot failsafe.
- Heartbeat watchdog between Raspberry Pi and Pixhawk.
- Low battery behavior: return-to-safe-point or hold position.

## 6) Recommended milestone plan

1. **M1**: Flood world loads + basic boat model control.
2. **M2**: Autonomous waypoint mission in static obstacles.
3. **M3**: Dynamic obstacle handling.
4. **M4**: SITL with PX4/ArduPilot and MAVROS bridge.
5. **M5**: On-water hardware tests with tether and manual backup.

