import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# pymavlink
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

def rotmat_from_quat(q):  # [x,y,z,w] -> 3x3
    x,y,z,w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy+zz), 2*(xy - wz),   2*(xz + wy)],
        [2*(xy + wz),   1 - 2*(xx+zz), 2*(yz - wx)],
        [2*(xz - wy),   2*(yz + wx),   1 - 2*(xx+yy)],
    ])

def quat_from_rotmat(R):
    # Robust conversion (numerically stable)
    m00,m01,m02 = R[0]; m10,m11,m12 = R[1]; m20,m21,m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr+1.0)*2
        w = 0.25*S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22)*2
        w = (m21 - m12) / S
        x = 0.25*S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22)*2
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25*S
        z = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11)*2
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25*S
    return np.array([x,y,z,w])

class VinsMavlinkBridge(Node):
    def __init__(self):
        super().__init__('vins_mavlink_bridge')

        # Params
        self.declare_parameter('odom_topic', '/vins/odometry')
        self.declare_parameter('serial_path', '/dev/serial/by-id/REPLACE_ME')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate_hz', 30)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.serial_path = self.get_parameter('serial_path').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().integer_value

        # MAVLink connection
        self.get_logger().info(f'Connecting MAVLink: {self.serial_path} @ {self.baud}')
        self.mav = mavutil.mavlink_connection(
            f'serial:{self.serial_path}',
            baud=self.baud, autoreconnect=True, dialect='ardupilotmega'
        )
        self.mav.wait_heartbeat(timeout=10)
        self.get_logger().info(f'Heartbeat OK: sysid={self.mav.target_system} compid={self.mav.target_component}')

        # ENU->NED rotation
        self.R_ENU_to_NED = np.array([[0,1,0],
                                      [1,0,0],
                                      [0,0,-1]], dtype=float)

        # Subscriber
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        # Rate limiter
        self.last_send = 0.0
        self.period = 1.0 / max(1, self.rate_hz)

    def enu_to_ned_vec(self, v):
        return self.R_ENU_to_NED @ v

    def enu_to_ned_quat(self, q_xyzw):
        R_enu = rotmat_from_quat(q_xyzw)
        R_ned = self.R_ENU_to_NED @ R_enu @ self.R_ENU_to_NED.T
        return quat_from_rotmat(R_ned)

    def odom_cb(self, msg: Odometry):
        now = time.time()
        if (now - self.last_send) < self.period:
            return
        self.last_send = now

        # Position (m)
        p_enu = np.array([msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z], dtype=float)
        p_ned = self.enu_to_ned_vec(p_enu)

        # Orientation (quaternion xyzw)
        q_enu = np.array([msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w], dtype=float)
        q_ned = self.enu_to_ned_quat(q_enu)

        # Velocity (m/s)
        v_enu = np.array([msg.twist.twist.linear.x,
                          msg.twist.twist.linear.y,
                          msg.twist.twist.linear.z], dtype=float)
        v_ned = self.enu_to_ned_vec(v_enu)

        # Angular rates: ENU -> NED (for small rates, same transform of vector)
        w_enu = np.array([msg.twist.twist.angular.x,
                          msg.twist.twist.angular.y,
                          msg.twist.twist.angular.z], dtype=float)
        w_ned = self.enu_to_ned_vec(w_enu)

        # Time (microseconds, UNIX)
        usec = int(now * 1e6)

        # Frame enums (MAV_FRAME)
        frame_id     = mavlink2.MAV_FRAME_LOCAL_NED          # 1
        child_frame  = mavlink2.MAV_FRAME_BODY_FRD           # 12 (forward-right-down body)

        # Covariances (optional, put -1 for unknown)
        cov_pose = [ -1.0 ] * 21
        cov_vel  = [ -1.0 ] * 21

        # Send MAVLink ODOMETRY
        self.mav.mav.odometry_send(
            usec,
            frame_id,
            child_frame,
            p_ned[0], p_ned[1], p_ned[2],
            q_ned[0], q_ned[1], q_ned[2], q_ned[3],
            v_ned[0], v_ned[1], v_ned[2],
            w_ned[0], w_ned[1], w_ned[2],
            cov_pose, cov_vel,
            mavlink2.MAV_ESTIMATOR_TYPE_VISION
        )

def main():
    rclpy.init()
    node = VinsMavlinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
