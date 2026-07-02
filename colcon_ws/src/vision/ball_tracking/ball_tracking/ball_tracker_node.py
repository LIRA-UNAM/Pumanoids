#!/usr/bin/env python3
"""
ROS2 node that filters noisy ball detections with a constant-velocity Kalman
filter and publishes a smoothed, outlier-rejected, occlusion-tolerant estimate.

Data flow
---------
  subscribe : geometry_msgs/msg/Pose2D  on  <input_topic>   (raw detections; x,y)
  publish   : geometry_msgs/msg/Pose2D  on  /vision/ball_kalmanized   (filtered)
  publish   : geometry_msgs/msg/PoseWithCovarianceStamped (optional, with cov)

Why a timer drives the filter
------------------------------
Detections arrive irregularly and stop completely while the ball is off-sight.
Rather than only stepping the filter when a detection appears, a fixed-rate timer
runs predict() + publish() at a steady rate; the subscription callback merely
stashes the newest detection. This yields continuous output that coasts through
occlusion and re-locks when the ball reappears. If the ball stays off-sight
longer than `max_coast_time`, the track is declared lost and reset so it
re-initializes cleanly on the next detection instead of coasting into garbage.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped

try:                                    # running as a plain script
    from ball_kalman import BallKalmanFilter
except ImportError:                     # running as an installed package module
    from .ball_kalman import BallKalmanFilter


class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker')

        # ---------------- parameters ----------------
        self.declare_parameter('input_topic', '/vision/map_ball')
        self.declare_parameter('output_topic', '/vision/ball_kal')
        self.declare_parameter('cov_topic', '/vision/ball_kalmanized_cov')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('process_accel_std', 6.0)
        self.declare_parameter('meas_pos_std', 0.4)
        self.declare_parameter('gate_threshold', 9.21)          # chi2(2dof), 99%
        self.declare_parameter('max_consecutive_rejections', 6)
        self.declare_parameter('max_coast_time', 1.0)           # s off-sight -> lost
        self.declare_parameter('publish_heading', True)         # theta = velocity dir
        self.declare_parameter('publish_covariance', True)
        self.declare_parameter('frame_id', 'pumas_map')

        g = self.get_parameter
        in_topic = g('input_topic').value
        out_topic = g('output_topic').value
        rate = float(g('publish_rate_hz').value)
        self.max_coast = float(g('max_coast_time').value)
        self.publish_heading = bool(g('publish_heading').value)
        self.publish_cov = bool(g('publish_covariance').value)
        self.frame_id = g('frame_id').value

        # ---------------- filter --------------------
        self.kf = BallKalmanFilter(
            dt=1.0 / rate,
            process_accel_std=float(g('process_accel_std').value),
            meas_pos_std=float(g('meas_pos_std').value),
            gate_threshold=float(g('gate_threshold').value),
            max_consecutive_rejections=int(g('max_consecutive_rejections').value),
        )

        # ---------------- shared state --------------
        self._lock = threading.Lock()
        self._pending_xy = None      # newest unconsumed detection (x, y)
        self._last_msg_time = None   # clock time of most recent detection
        self._last_time = None       # clock time of previous filter step
        self._last_theta = 0.0

        # ---------------- pub / sub -----------------
        # Detections: sensor-data QoS (best-effort) matches typical CV publishers
        # and is compatible with both reliable and best-effort sources.
        self.sub = self.create_subscription(
            Pose2D, in_topic, self._on_detection, qos_profile_sensor_data)

        out_qos = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Pose2D, out_topic, out_qos)

        self.cov_pub = None
        if self.publish_cov:
            self.cov_pub = self.create_publisher(
                PoseWithCovarianceStamped, g('cov_topic').value, out_qos)

        # ---------------- timer ---------------------
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"ball_tracker up: '{in_topic}' -> '{out_topic}' @ {rate:.0f} Hz "
            f"(gate={self.kf.gate:.2f}, coast<= {self.max_coast:.2f}s)")

    # ---- subscription callback: stash newest detection only ----
    def _on_detection(self, msg: Pose2D):
        now = self.get_clock().now()
        with self._lock:
            self._pending_xy = (msg.x, msg.y)
            self._last_msg_time = now

    # ---- timer: advance filter + publish at a steady rate ----
    def _on_timer(self):
        now = self.get_clock().now()

        with self._lock:
            z = self._pending_xy
            self._pending_xy = None
            last_msg_time = self._last_msg_time

        # Real elapsed time since the previous step (handles jitter/pauses).
        if self._last_time is None:
            dt = None
        else:
            dt = (now - self._last_time).nanoseconds * 1e-9
            dt = dt if dt > 0.0 else None
        self._last_time = now

        # Wait for the first detection before doing anything.
        if not self.kf.initialized and z is None:
            return

        # Off-sight too long? Declare the track lost and reset.
        if self.kf.initialized and last_msg_time is not None:
            gap = (now - last_msg_time).nanoseconds * 1e-9
            if gap > self.max_coast:
                self.get_logger().warn(
                    f"ball off-sight {gap:.2f}s > {self.max_coast:.2f}s: track lost")
                self.kf.reset()
                return

        pos = self.kf.update(z, dt=dt)   # predict (+ gated update if z is not None)
        if pos is None:
            return

        self._publish(pos, now)

    # ---- publishing ----
    def _publish(self, pos, stamp_time):
        out = Pose2D()
        out.x = float(pos[0])
        out.y = float(pos[1])
        if self.publish_heading:
            vx, vy = self.kf.velocity
            if math.hypot(vx, vy) > 1e-3:          # only trust heading when moving
                self._last_theta = math.atan2(vy, vx)
            out.theta = self._last_theta
        else:
            out.theta = 0.0
        self.pub.publish(out)

        if self.cov_pub is not None:
            self._publish_cov(pos, stamp_time)

    def _publish_cov(self, pos, stamp_time):
        m = PoseWithCovarianceStamped()
        m.header.stamp = stamp_time.to_msg()
        m.header.frame_id = self.frame_id
        m.pose.pose.position.x = float(pos[0])
        m.pose.pose.position.y = float(pos[1])
        m.pose.pose.orientation.w = 1.0

        # ROS covariance is a 6x6 row-major [x y z roll pitch yaw] block.
        # Fill the x/y sub-block from the filter; mark unused axes as unknown.
        P = self.kf.P
        cov = [0.0] * 36
        cov[0] = float(P[0, 0])   # var(x)
        cov[1] = float(P[0, 1])   # cov(x,y)
        cov[6] = float(P[1, 0])   # cov(y,x)
        cov[7] = float(P[1, 1])   # var(y)
        for idx in (14, 21, 28, 35):   # z, roll, pitch, yaw -> unknown
            cov[idx] = 1e6
        m.pose.covariance = cov
        self.cov_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
