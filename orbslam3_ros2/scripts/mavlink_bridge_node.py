#!/usr/bin/env python3
"""
ORB-SLAM3 to ArduSub MAVLink Bridge Node
========================================
Subscribes to PoseWithCovarianceStamped and SlamStatus from ORB-SLAM3 ROS 2 nodes,
transforms poses from ROS coordinates (FLU) to ArduPilot coordinates (NED / FRD),
and sends VISION_POSITION_ESTIMATE (#102) and/or VISION_POSITION_DELTA (#11011)
via MAVLink to ArduSub / ArduPilot.

Supported modes:
- 'estimate' : Transmits VISION_POSITION_ESTIMATE (MAVLink #102)
- 'delta'    : Transmits VISION_POSITION_DELTA (MAVLink #11011)
- 'both'     : Transmits both messages
"""

import os
import math
import sys
import time
from typing import Optional, Tuple

# Force MAVLink 2.0 and ArduPilotMega dialect for VISION_POSITION_DELTA (#11011) and reset_counter
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    from orbslam3_msgs.msg import SlamStatus
    HAS_SLAM_STATUS = True
except ImportError:
    HAS_SLAM_STATUS = False

try:
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as apm
except ImportError:
    print("ERROR: pymavlink is not installed! Install via 'pip3 install pymavlink'", file=sys.stderr)
    sys.exit(1)


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('orbslam3_mavlink_bridge')

        # Declare parameters
        self.declare_parameter('connection_url', 'udpin:0.0.0.0:14550') # tcp:127.0.0.1:5762
        self.declare_parameter('bridge_mode', 'both')  # 'estimate', 'delta', 'both'
        self.declare_parameter('source_system_id', 1)
        self.declare_parameter('source_component_id', 255)  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        self.declare_parameter('target_system_id', 1)
        self.declare_parameter('target_component_id', 1)
        self.declare_parameter('publish_rate_hz', 0.0)  # 0.0 = every incoming frame
        self.declare_parameter('min_confidence', 10.0)
        self.declare_parameter('default_confidence', 100.0)
        self.declare_parameter('max_delta_time_sec', 0.5)
        self.declare_parameter('max_delta_dist_m', 2.0)
        self.declare_parameter('heartbeat_rate_hz', 1.0)
        self.declare_parameter('input_is_flu', True)  # True = FLU (ROS standard), False = RDF (optical)

        # Retrieve parameters
        self.connection_url = str(self.get_parameter('connection_url').value)
        self.bridge_mode = str(self.get_parameter('bridge_mode').value).lower()
        self.source_system_id = int(self.get_parameter('source_system_id').value)
        self.source_component_id = int(self.get_parameter('source_component_id').value)
        self.target_system_id = int(self.get_parameter('target_system_id').value)
        self.target_component_id = int(self.get_parameter('target_component_id').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.default_confidence = float(self.get_parameter('default_confidence').value)
        self.max_delta_time_sec = float(self.get_parameter('max_delta_time_sec').value)
        self.max_delta_dist_m = float(self.get_parameter('max_delta_dist_m').value)
        self.heartbeat_rate_hz = float(self.get_parameter('heartbeat_rate_hz').value)
        self.input_is_flu = bool(self.get_parameter('input_is_flu').value)

        if self.bridge_mode not in ('estimate', 'delta', 'both'):
            self.get_logger().warn(
                f"Unknown bridge_mode '{self.bridge_mode}', defaulting to 'both'"
            )
            self.bridge_mode = 'both'

        # State tracking
        self.current_tracking_state: int = 2  # default TRACKING_OK
        self.current_confidence: float = self.default_confidence
        self.reset_counter: int = 0
        self.last_map_id: int = 0
        self.prev_time_sec: Optional[float] = None
        self.prev_pos_flu: Optional[np.ndarray] = None
        self.prev_rot_flu: Optional[np.ndarray] = None
        self.last_pub_time_sec: float = 0.0

        # FLU to NED / FRD transformation matrix (diag(1, -1, -1))
        # Forward -> Forward, Left -> -Right, Up -> -Down
        self.T_flu_to_ned = np.array([
            [ 1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0, -1.0]
        ], dtype=np.float64)

        # Connect to MAVLink
        self.mav = None
        self.connect_mavlink()

        # Heartbeat timer
        if self.heartbeat_rate_hz > 0.0:
            heartbeat_period = 1.0 / self.heartbeat_rate_hz
            self.heartbeat_timer = self.create_timer(heartbeat_period, self.send_heartbeat)

        # QoS configuration
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Pose subscriber
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/Passive/slam/pose_cov',
            self.pose_callback,
            qos
        )

        # Status subscriber (if available)
        if HAS_SLAM_STATUS:
            self.status_sub = self.create_subscription(
                SlamStatus,
                '/Passive/slam/slam_status',
                self.status_callback,
                qos
            )
            self.get_logger().info(f"Subscribed to slam status topic")
        else:
            self.get_logger().warn(
                "orbslam3_msgs.SlamStatus message not found. Confidence will use default value."
            )

        self.get_logger().info(
            f"ORB-SLAM3 MAVLink Bridge initialized.\n"
            f"  Mode: {self.bridge_mode}\n"
            f"  Endpoint: {self.connection_url}\n"
            f"  Source Component ID: {self.source_component_id}"
        )

    def connect_mavlink(self):
        """Establish MAVLink connection with auto-reconnect fallback."""
        try:
            self.get_logger().info(f"Connecting to MAVLink at {self.connection_url}...")
            self.mav = mavutil.mavlink_connection(
                self.connection_url,
                dialect='ardupilotmega',
                source_system=200,
                source_component=apm.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
            )
            self.get_logger().info("MAVLink connection established.")
            self.get_logger().info(f'Waiting for heartbeat on {self.connection_url}...')
            self.mav.wait_heartbeat()
            self.get_logger().info('Heartbeat received from system (system %u component %u)' %
                               (self.mav.target_system, self.mav.target_component))
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink ({self.connection_url}): {e}")
            self.mav = None

    def send_heartbeat(self):
        """Send periodic 1Hz MAVLink heartbeat."""
        if not self.mav:
            self.connect_mavlink()
            return

        try:
            self.mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,  # base_mode
                0,  # custom_mode
                mavutil.mavlink.MAV_STATE_ACTIVE
            )
        except Exception as e:
            self.get_logger().warn(f"Heartbeat send failed: {e}")

    def status_callback(self, msg: 'SlamStatus'):
        """Monitor SLAM tracking state and map modifications for reset tracking."""
        prev_state = self.current_tracking_state
        self.current_tracking_state = msg.tracking_state

        # Tracking state mapping:
        # -1 = NOT_READY, 0 = NO_IMAGES, 1 = NOT_INITIALIZED, 2 = OK, 3 = RECENTLY_LOST, 4 = LOST, 5 = OK_KLT
        if msg.tracking_state in (2,):
            self.current_confidence = 100.0
        elif msg.tracking_state in (5,):
            self.current_confidence = 80.0
        elif msg.tracking_state in (3,):
            self.current_confidence = 20.0
        else:
            self.current_confidence = 0.0

        # Check for loop closure, bundle adjustment, or relocalization reset
        reset_triggered = False
        if msg.map_changed:
            reset_triggered = True
            self.get_logger().info("SLAM map changed (loop closure/BA detected).")

        if msg.map_id != self.last_map_id:
            reset_triggered = True
            self.last_map_id = msg.map_id
            self.get_logger().info(f"SLAM active map changed to map_id: {msg.map_id}")

        if prev_state in (4, -1, 0, 1) and msg.tracking_state in (2, 5):
            reset_triggered = True
            self.get_logger().info("SLAM recovered tracking from lost state.")

        if reset_triggered:
            self.reset_counter = (self.reset_counter + 1) % 256
            # Reset delta reference pose to prevent jump delta
            self.prev_time_sec = None
            self.prev_pos_flu = None
            self.prev_rot_flu = None

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Main callback for receiving and forwarding poses."""
        if not self.mav:
            self.connect_mavlink()
            if not self.mav:
                return

        # Rate throttle check
        now_sec = time.time()
        if self.publish_rate_hz > 0.0:
            if (now_sec - self.last_pub_time_sec) < (1.0 / self.publish_rate_hz):
                return
        self.last_pub_time_sec = now_sec

        # Extract timestamp in microseconds (from ROS message stamp)
        stamp = msg.header.stamp
        time_usec = stamp.sec * 1_000_000 + stamp.nanosec // 1_000
        if time_usec == 0:
            time_usec = int(now_sec * 1_000_000)

        t_sec = stamp.sec + stamp.nanosec * 1e-9

        # Extract position in FLU (Forward-Left-Up)
        p = msg.pose.pose.position
        pos_flu = np.array([p.x, p.y, p.z], dtype=np.float64)

        # Extract orientation in FLU
        q = msg.pose.pose.orientation
        rot_flu = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

        # 1. Process VISION_POSITION_ESTIMATE
        if self.bridge_mode in ('estimate', 'both'):
            self.send_vision_position_estimate(time_usec, pos_flu, rot_flu, msg.pose.covariance)

        # 2. Process VISION_POSITION_DELTA
        if self.bridge_mode in ('delta', 'both'):
            self.send_vision_position_delta(time_usec, t_sec, pos_flu, rot_flu)

    def send_vision_position_estimate(
        self,
        time_usec: int,
        pos_flu: np.ndarray,
        rot_flu: np.ndarray,
        ros_covariance
    ):
        """Convert pose to NED and publish VISION_POSITION_ESTIMATE (#102)."""
        # Transform position to NED:
        # x_ned = x_flu (Forward)
        # y_ned = -y_flu (East / Right)
        # z_ned = -z_flu (Down)
        pos_ned = self.T_flu_to_ned @ pos_flu

        # Transform rotation to NED:
        # R_ned = M * R_flu * M^T
        rot_ned = self.T_flu_to_ned @ rot_flu @ self.T_flu_to_ned.T

        # Extract Euler angles in NED (Tait-Bryan Z-Y-X: Yaw -> Pitch -> Roll)
        roll, pitch, yaw = self.rotation_matrix_to_euler_ned(rot_ned)

        # Format 21-element upper-triangle covariance
        cov_21 = self.pack_covariance_21(ros_covariance)

        try:
            self.mav.mav.vision_position_estimate_send(
                time_usec,
                float(pos_ned[0]),
                float(pos_ned[1]),
                float(pos_ned[2]),
                float(roll),
                float(pitch),
                float(yaw),
                cov_21,
                int(self.reset_counter)
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send VISION_POSITION_ESTIMATE: {e}")

    def send_vision_position_delta(
        self,
        time_usec: int,
        t_sec: float,
        pos_flu: np.ndarray,
        rot_flu: np.ndarray
    ):
        """Compute relative body-frame (FRD) delta and publish VISION_POSITION_DELTA (#11011)."""
        if self.prev_time_sec is None or self.prev_pos_flu is None or self.prev_rot_flu is None:
            # Initialize baseline
            self.prev_time_sec = t_sec
            self.prev_pos_flu = pos_flu
            self.prev_rot_flu = rot_flu
            return

        dt = t_sec - self.prev_time_sec
        # Guard against backwards or stalled timestamps
        if dt <= 0.0:
            return

        # If time gap is too large, reset baseline and skip to prevent huge velocity spike
        if dt > self.max_delta_time_sec:
            self.get_logger().warn(
                f"Delta time gap too large ({dt:.3f}s > {self.max_delta_time_sec}s). Resetting delta baseline."
            )
            self.prev_time_sec = t_sec
            self.prev_pos_flu = pos_flu
            self.prev_rot_flu = rot_flu
            return

        # Check for abnormal jump
        dist = np.linalg.norm(pos_flu - self.prev_pos_flu)
        if dist > self.max_delta_dist_m:
            self.get_logger().warn(
                f"Abnormal position jump detected ({dist:.2f}m > {self.max_delta_dist_m}m). Triggering reset."
            )
            self.reset_counter = (self.reset_counter + 1) % 256
            self.prev_time_sec = t_sec
            self.prev_pos_flu = pos_flu
            self.prev_rot_flu = rot_flu
            return

        time_delta_usec = int(dt * 1_000_000)

        # 1. Position delta in previous body frame (FLU):
        # delta_p_body_flu = R_prev^T * (pos_curr - pos_prev)
        delta_p_body_flu = self.prev_rot_flu.T @ (pos_flu - self.prev_pos_flu)

        # Convert body FLU delta to body FRD delta:
        # dx_frd = dx_flu
        # dy_frd = -dy_flu
        # dz_frd = -dz_flu
        delta_p_frd = np.array([
            delta_p_body_flu[0],
            -delta_p_body_flu[1],
            -delta_p_body_flu[2]
        ], dtype=np.float64)

        # 2. Rotation delta in previous body frame:
        # delta_R_flu = R_prev^T * rot_curr
        delta_R_flu = self.prev_rot_flu.T @ rot_flu

        # Transform delta rotation to FRD:
        # delta_R_frd = M * delta_R_flu * M^T
        delta_R_frd = self.T_flu_to_ned @ delta_R_flu @ self.T_flu_to_ned.T

        # Extract delta Euler angles [d_roll, d_pitch, d_yaw] in radians
        d_roll, d_pitch, d_yaw = self.rotation_matrix_to_euler_ned(delta_R_frd)
        angle_delta = [float(d_roll), float(d_pitch), float(d_yaw)]
        position_delta = [float(delta_p_frd[0]), float(delta_p_frd[1]), float(delta_p_frd[2])]

        # Skip delta if confidence is below minimum threshold
        confidence = float(self.current_confidence)
        if confidence < self.min_confidence:
            # Still update baseline so we don't build up a large delta
            self.prev_time_sec = t_sec
            self.prev_pos_flu = pos_flu
            self.prev_rot_flu = rot_flu
            return

        try:
            self.mav.mav.vision_position_delta_send(
                time_usec,
                time_delta_usec,
                angle_delta,
                position_delta,
                confidence
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send VISION_POSITION_DELTA: {e}")

        # Update baseline
        self.prev_time_sec = t_sec
        self.prev_pos_flu = pos_flu
        self.prev_rot_flu = rot_flu

    @staticmethod
    def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        """Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix."""
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm > 1e-9:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
        else:
            return np.identity(3, dtype=np.float64)

        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)]
        ], dtype=np.float64)

    @staticmethod
    def rotation_matrix_to_euler_ned(R: np.ndarray) -> Tuple[float, float, float]:
        """
        Extract Tait-Bryan Z-Y-X Euler angles (Roll, Pitch, Yaw) from rotation matrix in NED.
        Convention: Yaw (around Z), Pitch (around Y), Roll (around X).
        """
        sin_pitch = -float(R[2, 0])
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)

        if abs(abs(sin_pitch) - 1.0) > 1e-6:
            roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
            yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        else:
            roll = 0.0
            yaw = math.atan2(-float(R[0, 1]), float(R[1, 1]))

        return roll, pitch, yaw

    @staticmethod
    def pack_covariance_21(cov_36) -> list:
        """
        Extract 21-element upper-triangular array from 6x6 (36 elements) ROS covariance.
        Order of 6D state: [x, y, z, roll, pitch, yaw]
        """
        cov_21 = [0.0] * 21
        if cov_36 is None or len(cov_36) < 36:
            return cov_21

        # Check if SLAM indicated invalid/lost (-1 on diagonal)
        if cov_36[0] < 0:
            return cov_21

        idx = 0
        for i in range(6):
            for j in range(i, 6):
                val = float(cov_36[i * 6 + j])
                cov_21[idx] = val if not math.isnan(val) else 0.0
                idx += 1

        return cov_21


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
