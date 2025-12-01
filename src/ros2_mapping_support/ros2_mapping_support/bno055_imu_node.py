#!/usr/bin/env python3
import math
import time
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import smbus

# ==== BNO055 constants ====
BNO055_CHIP_ADDR       = 0x28  # default, overridden by param
BNO055_PAGE_ID         = 0x07
BNO055_CHIP_ID         = 0x00

BNO055_OPR_MODE        = 0x3D
BNO055_POWER_MODE      = 0x3E
BNO055_SYS_TRIGGER     = 0x3F
BNO055_UNIT_SEL        = 0x3B

BNO055_CALIB_STAT      = 0x35

# Data registers
BNO055_QUATERNION_DATA_W_LSB = 0x20
BNO055_GYRO_DATA_X_LSB       = 0x14
BNO055_ACC_DATA_X_LSB        = 0x08

# Operation modes
OPR_MODE_CONFIG        = 0x00
OPR_MODE_IMUPLUS       = 0x08   # gyro + accel fusion (no mag)
OPR_MODE_NDOF          = 0x0C   # only used when generating offsets

# Offset registers
BNO055_OFFSET_START    = 0x55
BNO055_OFFSET_END      = 0x6A   # inclusive


def write8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val & 0xFF)


def read8(bus, addr, reg):
    return bus.read_byte_data(addr, reg) & 0xFF


def read_s16(bus, addr, reg):
    lo = read8(bus, addr, reg)
    hi = read8(bus, addr, reg + 1)
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v


def read_vector_s16(bus, addr, start_reg, length):
    # length in "components" (x,y,z) etc., each is 2 bytes
    vals = []
    for i in range(length):
        vals.append(read_s16(bus, addr, start_reg + 2 * i))
    return vals


def set_mode(bus, addr, mode):
    write8(bus, addr, BNO055_OPR_MODE, mode)
    time.sleep(0.03)


def set_units_and_power(bus, addr):
    # UNIT_SEL: 0x00 => deg, deg/s, m/s^2, Celsius, Android orientation
    write8(bus, addr, BNO055_UNIT_SEL, 0x00)
    # NORMAL power
    write8(bus, addr, BNO055_POWER_MODE, 0x00)
    # PAGE 0
    write8(bus, addr, BNO055_PAGE_ID, 0x00)
    time.sleep(0.05)


def load_offsets_from_file(path):
    if not os.path.isfile(path):
        return None
    with open(path, "r") as f:
        return json.load(f)


def write_offsets_to_bno(bus, addr, offs_dict):
    """
    offs_dict keys:
      acc_offset_x/y/z, mag_offset_x/y/z,
      gyro_offset_x/y/z, acc_radius, mag_radius
    """
    if offs_dict is None:
        return

    def u16(v):
        if v < 0:
            v = (1 << 16) + v
        return v & 0xFFFF

    # Pack into 22-byte buffer in datasheet order
    vals = [
        u16(offs_dict["acc_offset_x"]),
        u16(offs_dict["acc_offset_y"]),
        u16(offs_dict["acc_offset_z"]),
        u16(offs_dict["gyro_offset_x"]),
        u16(offs_dict["gyro_offset_y"]),
        u16(offs_dict["gyro_offset_z"]),
        u16(offs_dict["acc_radius"]),
    ]

    bytes_out = []
    for v in vals:
        bytes_out.append(v & 0xFF)       # LSB
        bytes_out.append((v >> 8) & 0xFF)  # MSB

    # Write as I2C block
    bus.write_i2c_block_data(addr, BNO055_OFFSET_START, bytes_out)


def quat_to_yaw(w, x, y, z):
    # Standard yaw (Z) from quaternion
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(z_yaw):
    # pure Z rotation quaternion
    half = z_yaw * 0.5
    cz = math.cos(half)
    sz = math.sin(half)
    # rotation about Z only
    return (cz, 0.0, 0.0, sz)


def quat_multiply(q1, q2):
    # (w1,x1,y1,z1)*(w2,x2,y2,z2)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (w, x, y, z)


class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__("bno055_imu_node")

        # Parameters
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_address", 0x28)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate", 50.0)  # Hz
        self.declare_parameter("calib_file", os.path.expanduser("~/.ros/bno055_offsets.json"))
        self.declare_parameter("use_stored_offsets", True)
        self.declare_parameter("zero_yaw_on_start", True)
        self.declare_parameter("warmup_seconds", 3.0)

        self.i2c_bus_num = int(self.get_parameter("i2c_bus").value)
        self.i2c_addr    = int(self.get_parameter("i2c_address").value)
        self.frame_id    = str(self.get_parameter("frame_id").value)
        self.pub_rate    = float(self.get_parameter("publish_rate").value)
        self.calib_file  = str(self.get_parameter("calib_file").value)
        self.use_offsets = bool(self.get_parameter("use_stored_offsets").value)
        self.zero_yaw    = bool(self.get_parameter("zero_yaw_on_start").value)
        self.warmup_sec  = float(self.get_parameter("warmup_seconds").value)

        self.get_logger().info(f"Using BNO055 on I2C bus {self.i2c_bus_num}, addr 0x{self.i2c_addr:02X}")
        self.get_logger().info(f"Frame id: {self.frame_id}")
        self.get_logger().info(f"Offsets file: {self.calib_file}")

        # Open I2C
        self.bus = smbus.SMBus(self.i2c_bus_num)

        # Init BNO
        self._init_bno()

        # Zero-yaw state
        self.yaw_offset = None
        self.start_time = time.time()

        # Publisher
        self.pub = self.create_publisher(Imu, "/imu/data", 10)

        # Timer
        dt = 1.0 / self.pub_rate
        self.timer = self.create_timer(dt, self._timer_cb)

    # ----------------- hardware init -----------------

    def _init_bno(self):
        # CONFIG mode
        self.get_logger().info("Putting BNO055 into CONFIG mode...")
        set_mode(self.bus, self.i2c_addr, OPR_MODE_CONFIG)
        set_units_and_power(self.bus, self.i2c_addr)

        # Optionally load & write stored offsets
        if self.use_offsets:
            offs = load_offsets_from_file(self.calib_file)
            if offs is None:
                self.get_logger().warn("No stored offsets file found, running without digital calibration.")
            else:
                self.get_logger().info("Writing stored calibration offsets into BNO055...")
                write_offsets_to_bno(self.bus, self.i2c_addr, offs)
        else:
            self.get_logger().info("use_stored_offsets=False, skipping offset write.")

        # Start in IMUPLUS (gyro+acc, no magnetometer)
        self.get_logger().info("Switching BNO055 to IMUPLUS fusion mode (no mag)...")
        set_mode(self.bus, self.i2c_addr, OPR_MODE_IMUPLUS)

    # ----------------- main loop -----------------

    def _timer_cb(self):
        now = time.time()

        # Warmup: just consume data for a few seconds to let gyro bias settle
        if now - self.start_time < self.warmup_sec:
            _ = self._read_imu_raw()
            return

        quat, ang_vel, lin_acc = self._read_imu_raw()
        if quat is None:
            return

        w, x, y, z = quat

        # Compute yaw from raw quaternion
        yaw = quat_to_yaw(w, x, y, z)

        # First sample: capture yaw_offset so that this pose becomes yaw=0
        if self.zero_yaw and self.yaw_offset is None:
            self.yaw_offset = yaw
            self.get_logger().info(f"Zeroing yaw. Initial yaw={math.degrees(yaw):.1f} deg → 0 deg.")

        if self.yaw_offset is not None:
            # Apply yaw correction by multiplying with inverse of yaw_offset
            # q_correct = q * q_z(-yaw_offset)
            q_corr = yaw_to_quat(-self.yaw_offset)
            w, x, y, z = quat_multiply((w, x, y, z), q_corr)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.w = float(w)
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)

        # Covariances: simple diagonal, tweak if you want
        msg.orientation_covariance[0] = 0.0025
        msg.orientation_covariance[4] = 0.0025
        msg.orientation_covariance[8] = 0.0025

        # Angular velocity: BNO gives deg/s, but we configured UNIT_SEL=0x00 (deg/s),
        # convert to rad/s
        avx, avy, avz = ang_vel
        to_rad = math.radians(1.0)
        msg.angular_velocity.x = avx * to_rad
        msg.angular_velocity.y = avy * to_rad
        msg.angular_velocity.z = avz * to_rad

        msg.angular_velocity_covariance[0] = 0.0004
        msg.angular_velocity_covariance[4] = 0.0004
        msg.angular_velocity_covariance[8] = 0.0004

        # Linear acceleration: m/s^2 already (UNIT_SEL=0x00)
        lax, lay, laz = lin_acc
        msg.linear_acceleration.x = float(lax)
        msg.linear_acceleration.y = float(lay)
        msg.linear_acceleration.z = float(laz)

        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self.pub.publish(msg)

    def _read_imu_raw(self):
        try:
            # Quaternion: 4 * s16, scale factor 1/2^14
            qw = read_s16(self.bus, self.i2c_addr, BNO055_QUATERNION_DATA_W_LSB + 0)
            qx = read_s16(self.bus, self.i2c_addr, BNO055_QUATERNION_DATA_W_LSB + 2)
            qy = read_s16(self.bus, self.i2c_addr, BNO055_QUATERNION_DATA_W_LSB + 4)
            qz = read_s16(self.bus, self.i2c_addr, BNO055_QUATERNION_DATA_W_LSB + 6)
            scale = 1.0 / (1 << 14)
            qw_f = qw * scale
            qx_f = qx * scale
            qy_f = qy * scale
            qz_f = qz * scale

            # Gyro: deg/s, 16 LSB/deg/s (datasheet)
            gx = read_s16(self.bus, self.i2c_addr, BNO055_GYRO_DATA_X_LSB + 0) / 16.0
            gy = read_s16(self.bus, self.i2c_addr, BNO055_GYRO_DATA_X_LSB + 2) / 16.0
            gz = read_s16(self.bus, self.i2c_addr, BNO055_GYRO_DATA_X_LSB + 4) / 16.0

            # Accel: m/s^2, 100 LSB/(m/s^2)
            ax = read_s16(self.bus, self.i2c_addr, BNO055_ACC_DATA_X_LSB + 0) / 100.0
            ay = read_s16(self.bus, self.i2c_addr, BNO055_ACC_DATA_X_LSB + 2) / 100.0
            az = read_s16(self.bus, self.i2c_addr, BNO055_ACC_DATA_X_LSB + 4) / 100.0

            return (qw_f, qx_f, qy_f, qz_f), (gx, gy, gz), (ax, ay, az)
        except Exception as e:
            self.get_logger().warn(f"Failed to read BNO055 data: {e}")
            return (None, None, None)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
