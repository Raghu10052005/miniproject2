#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class MissionState(str, Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    AVOIDING = 'AVOIDING'
    COMPLETE = 'COMPLETE'


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')

        self.declare_parameter('control_rate_hz', 5.0)
        self.declare_parameter('arrival_tolerance_m', 2.0)
        self.declare_parameter('max_linear_speed', 1.2)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('obstacle_threshold_m', 4.0)
        self.declare_parameter('avoid_turn_speed', 0.4)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.arrival_tolerance_m = float(self.get_parameter('arrival_tolerance_m').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.obstacle_threshold_m = float(self.get_parameter('obstacle_threshold_m').value)
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)

        self.state: MissionState = MissionState.IDLE
        self.waypoints: List[Tuple[float, float]] = []
        self.current_wp_idx: int = 0

        self.current_position_xy: Optional[Tuple[float, float]] = None
        self.current_yaw_rad: float = 0.0
        self.last_obstacle_distance: float = 999.0

        self.origin_lat_lon: Optional[Tuple[float, float]] = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_waypoint', 10)

        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Float32, '/obstacle_distance_m', self.obstacle_cb, 10)
        self.create_subscription(PoseArray, '/mission/waypoints', self.waypoints_cb, 10)

        period = 1.0 / max(self.control_rate_hz, 1e-3)
        self.create_timer(period, self.control_loop)

        self.get_logger().info('mission_node started.')

    def gps_cb(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.origin_lat_lon is None:
            self.origin_lat_lon = (msg.latitude, msg.longitude)

        self.current_position_xy = self.latlon_to_local_xy(msg.latitude, msg.longitude)

    def imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    def obstacle_cb(self, msg: Float32) -> None:
        self.last_obstacle_distance = float(msg.data)

    def waypoints_cb(self, msg: PoseArray) -> None:
        new_waypoints: List[Tuple[float, float]] = []
        for pose in msg.poses:
            new_waypoints.append((float(pose.position.x), float(pose.position.y)))

        if not new_waypoints:
            self.get_logger().warning('Received empty waypoint list.')
            return

        self.waypoints = new_waypoints
        self.current_wp_idx = 0
        self.state = MissionState.NAVIGATING
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints. Starting mission.')

    def control_loop(self) -> None:
        self.publish_state()

        if self.state == MissionState.IDLE:
            self.stop_boat()
            return

        if self.state == MissionState.COMPLETE:
            self.stop_boat()
            return

        if self.current_position_xy is None or not self.waypoints:
            self.stop_boat()
            return

        if self.last_obstacle_distance < self.obstacle_threshold_m:
            self.state = MissionState.AVOIDING
        elif self.state == MissionState.AVOIDING:
            self.state = MissionState.NAVIGATING

        if self.state == MissionState.AVOIDING:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.avoid_turn_speed
            self.cmd_pub.publish(cmd)
            return

        target = self.waypoints[self.current_wp_idx]
        self.publish_target_waypoint(target)

        dist = self.distance(self.current_position_xy, target)
        if dist < self.arrival_tolerance_m:
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.state = MissionState.COMPLETE
                self.get_logger().info('Mission complete.')
                self.stop_boat()
                return
            target = self.waypoints[self.current_wp_idx]
            dist = self.distance(self.current_position_xy, target)

        cmd = self.compute_simple_pursuit_cmd(self.current_position_xy, self.current_yaw_rad, target, dist)
        self.cmd_pub.publish(cmd)

    def compute_simple_pursuit_cmd(
        self, current_xy: Tuple[float, float], yaw: float, target_xy: Tuple[float, float], dist: float
    ) -> Twist:
        dx = target_xy[0] - current_xy[0]
        dy = target_xy[1] - current_xy[1]

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - yaw)

        cmd = Twist()
        cmd.angular.z = self.clamp(1.2 * heading_error, -self.max_angular_speed, self.max_angular_speed)

        heading_gain = max(0.0, 1.0 - min(abs(heading_error), math.pi) / math.pi)
        distance_gain = min(1.0, dist / 10.0)
        cmd.linear.x = self.max_linear_speed * heading_gain * distance_gain
        return cmd

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def publish_target_waypoint(self, target: Tuple[float, float]) -> None:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = Pose()
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = 0.0
        self.target_pub.publish(msg)

    def stop_boat(self) -> None:
        self.cmd_pub.publish(Twist())

    def latlon_to_local_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        if self.origin_lat_lon is None:
            return (0.0, 0.0)

        lat0, lon0 = self.origin_lat_lon
        r_earth = 6378137.0

        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)

        x = r_earth * dlon * math.cos(math.radians((lat + lat0) * 0.5))
        y = r_earth * dlat
        return (x, y)

    @staticmethod
    def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
