#!/usr/bin/env python3
import sys, termios, tty, subprocess, threading, time, os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
WASD teleop  (/cmd_vel) + Cartographer control
----------------------------------------------
  W : forward          S : backward
  A : turn left        D : turn right
  Space : stop         Q : quit

  R : Cartographer:
        - if not running  -> START mapping
        - if running      -> STOP, wait 10s, RESTART (reset map)

  1/2 : linear speed -/+ (default 0.30 m/s)
  9/0 : angular speed -/+ (default 1.20 rad/s)
"""

# Command to launch your Cartographer stack (lidar + IMU + TF + cartographer)
CARTO_LAUNCH_CMD = [
    "ros2", "launch", "ros2_mapping_support", "cartographer_lidar_imu.launch.py",
    "lidar_port:=/dev/rplidar",
    "i2c_bus:=7",
    "i2c_address:=0x28",
]


def getch():
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        return ''  # ignore if not a TTY
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = 0.30
        self.ang = 1.20

        # Process handle for the ros2 launch that runs Cartographer
        self.carto_proc = None
        self.reset_in_progress = False

        self.get_logger().info(HELP)
        self.get_logger().info(
            "Teleop ready. This script can also start/stop Cartographer with 'R'."
        )
        self.get_logger().info(
            "Tip: move the rover with WASD while it's off the ground, "
            "then press 'R' when it is on the floor to start mapping."
        )

    # ----------------- Cartographer control helpers -----------------

    def start_cartographer(self):
        # If already running, do nothing
        if self.carto_proc is not None and self.carto_proc.poll() is None:
            self.get_logger().info("Cartographer already running; not starting again.")
            return
        self.get_logger().info(
            "Starting Cartographer launch:\n  " + " ".join(CARTO_LAUNCH_CMD)
        )
        try:
            # Launch the full mapping stack (lidar + imu + tf + cartographer)
            self.carto_proc = subprocess.Popen(CARTO_LAUNCH_CMD)
        except Exception as e:
            self.get_logger().error(f"Failed to start Cartographer launch: {e}")
            self.carto_proc = None

    def stop_cartographer(self):
        # Stop the ros2 launch process we own (if any)
        if self.carto_proc is not None and self.carto_proc.poll() is None:
            self.get_logger().info("Stopping Cartographer launch (owned process)...")
            self.carto_proc.terminate()
            try:
                self.carto_proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Cartographer launch did not exit, killing...")
                self.carto_proc.kill()
            self.carto_proc = None

        # Backup: kill any remaining cartographer_node processes
        try:
            subprocess.run(
                ["pkill", "-f", "cartographer_ros.*cartographer_node"],
                check=False,
            )
        except Exception as e:
            self.get_logger().warn(f"pkill backup for cartographer_node failed: {e}")

    def reset_cartographer_with_delay(self, delay_sec: float = 10.0):
        """Stop Cartographer, wait delay_sec, then restart it."""
        if self.reset_in_progress:
            self.get_logger().warn("Cartographer reset already in progress, ignoring.")
            return

        self.reset_in_progress = True

        def worker():
            try:
                # If Cartographer is not running yet, just start it immediately
                if self.carto_proc is None or self.carto_proc.poll() is not None:
                    self.get_logger().info(
                        "Cartographer not running. Starting it now (no delay)."
                    )
                    self.start_cartographer()
                    return

                # Otherwise, do a full reset
                self.get_logger().info(
                    f"Cartographer reset requested. Stopping Cartographer and "
                    f"waiting {delay_sec:.1f} s before restart..."
                )
                # Stop Cartographer immediately
                self.stop_cartographer()
                # Wait so you can reposition robot, set it on the ground, etc.
                time.sleep(delay_sec)
                # Restart Cartographer
                self.start_cartographer()
                self.get_logger().info(
                    "Cartographer restarted. New map is being built from scratch."
                )
            finally:
                self.reset_in_progress = False

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    # ----------------- Teleop logic -----------------

    def _publish(self, vx, wz, note=''):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)
        if note:
            self.get_logger().info(note)

    def spin_keys(self):
        self._publish(0.0, 0.0, "ready: WASD, space=stop, R=Cartographer control, Q=quit")
        while rclpy.ok():
            c = getch()
            if not c:
                continue

            c_lower = c.lower()
            self.get_logger().debug(f"Key pressed: {repr(c)}")

            if   c_lower == 'w':
                self._publish(self.lin, 0.0, f"cmd: forward  v={self.lin:.2f}")
            elif c_lower == 's':
                self._publish(-self.lin, 0.0, f"cmd: back     v={-self.lin:.2f}")
            elif c_lower == 'a':
                self._publish(0.0, self.ang, f"cmd: left     w={self.ang:.2f}")
            elif c_lower == 'd':
                self._publish(0.0, -self.ang, f"cmd: right    w={-self.ang:.2f}")
            elif c == ' ':
                self._publish(0.0, 0.0, "cmd: stop")

            # Speed tweaks
            elif c == '1':
                self.lin = max(0.05, round(self.lin - 0.05, 3))
                self.get_logger().info(f"linear = {self.lin:.2f} m/s")
            elif c == '2':
                self.lin = min(1.50, round(self.lin + 0.05, 3))
                self.get_logger().info(f"linear = {self.lin:.2f} m/s")
            elif c == '9':
                self.ang = max(0.20, round(self.ang - 0.10, 3))
                self.get_logger().info(f"angular = {self.ang:.2f} rad/s")
            elif c == '0':
                self.ang = min(4.00, round(self.ang + 0.10, 3))
                self.get_logger().info(f"angular = {self.ang:.2f} rad/s")

            # --- Cartographer reset/start hotkey ---
            elif c_lower == 'r':
                self._publish(0.0, 0.0, "cmd: stop (Cartographer reset/start requested)")
                # If not running -> start immediately; if running -> stop + wait + restart
                self.reset_cartographer_with_delay(delay_sec=10.0)

            elif c_lower == 'q':
                self.get_logger().info("Quit requested, stopping teleop.")
                break


def main():
    rclpy.init()
    node = WasdTeleop()
    try:
        node.spin_keys()
    finally:
        # final stop for safety
        msg = Twist()
        node.pub.publish(msg)
        node.stop_cartographer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
