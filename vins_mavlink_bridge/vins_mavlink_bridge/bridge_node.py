#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

from pymavlink import mavutil


def enu_to_ned_pos(x, y, z):
    # ENU (E,N,U) -> NED (N,E,D)
    return y, x, -z


class VinsMavlinkBridge(Node):
    def __init__(self):
        super().__init__('vins_mavlink_bridge')

        # ---- Parameters
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('serial_path', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate_hz', 30)

        self.odom_topic: str = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.serial_path: str = self.get_parameter('serial_path').get_parameter_value().string_value
        self.baudrate: int = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.rate_hz: int = self.get_parameter('rate_hz').get_parameter_value().integer_value

        # ---- Open MAVLink (serial by default)
        device = self.serial_path
        # Force serial backend if you passed a plain /dev/ path
        if device.startswith('/dev/'):
            device = f'serial:{device}'

        self.get_logger().info(
            f'Connecting MAVLink (serial): {self.serial_path} @ {self.baudrate}'
        )
        self.mav = mavutil.mavserial(
            device=self.serial_path,   # e.g. /dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_...-if00
            baud=self.baudrate,
            source_system=1,
            source_component=191,
            autoreconnect=True
        )

        # Optional: wait briefly for heartbeat (don’t block forever)
        try:
            self.mav.wait_heartbeat(timeout=3)
            self.get_logger().info('MAVLink heartbeat received.')
        except Exception:
            self.get_logger().warn('No heartbeat yet; continuing...')

        # ---- ROS subscriber
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)

        # Rate limiting
        self._last_send = 0.0
        self._min_dt = 1.0 / max(1, self.rate_hz)

    def odom_cb(self, msg: Odometry):
        now = time.time()
        if now - self._last_send < self._min_dt:
            return
        self._last_send = now

        # ENU -> NED quick position conversion
        x_e, y_e, z_u = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        x_n, y_east, z_d = y_e, x_e, -z_u  # N, E, D

        # TODO: replace with real ENU->NED quaternion conversion
        q = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]

        vx = vy = vz = 0.0
        rollspeed = pitchspeed = yawspeed = 0.0

        # MAVLink expects 21-length cov arrays; NaNs mean "unknown"
        nan21 = [float('nan')] * 21

        # Timestamp in microseconds
        stamp_us = self.get_clock().now().nanoseconds // 1000

        try:
            self.mav.mav.odometry_send(
                stamp_us,
                1,      # frame_id: MAV_FRAME_LOCAL_NED = 1
                12,     # child_frame_id: MAV_FRAME_BODY_FRD = 12 (body Forward-Right-Down)
                x_n, y_east, z_d,
                q,      # quaternion [w, x, y, z]
                vx, vy, vz,
                rollspeed, pitchspeed, yawspeed,
                nan21,  # pose_covariance (len=21)
                nan21,  # velocity_covariance (len=21)
                0,      # reset_counter
                3       # estimator_type: 3 = VISION
            )
        except Exception as e:
            self.get_logger().warn(f"ODOMETRY send failed: {e}")


def main():
    rclpy.init()
    node = VinsMavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

