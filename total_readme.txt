====================
File: MapReset.py
====================
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import time


class TabletResetRunner(Node):
    def __init__(self):
        super().__init__('tablet_reset_runner')

        # Path to your reset bash script
        self.reset_cmd = "bash ~/reset_cartographer.sh"  # <-- Edit this path if needed

        self.sub = self.create_subscription(
            Empty,
            '/tablet_reset_slam',  # Topic you're already publishing to
            self.on_reset,
            10
        )

        self.get_logger().info(
            'TabletResetRunner listening on /tablet_reset_slam (std_msgs/Empty).'
        )
        self.get_logger().info(f'Configured reset_cmd: {self.reset_cmd}')

    def on_reset(self, msg: Empty):
        self.get_logger().info(f'Reset requested – running: {self.reset_cmd}')

        # Start by killing any existing Cartographer nodes and restart them
        try:
            # Run the bash reset script in a login shell
            subprocess.Popen(
                ['bash', '-lc', self.reset_cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info('Cartographer reset command triggered.')
        except Exception as e:
            self.get_logger().error(f'Failed to run reset_cmd: {e}')

        # Optional: sleep to allow Cartographer time to restart
        time.sleep(2)

        # Additional reset logic from your script: kill Cartographer nodes
        self.get_logger().info("[reset_cartographer] Killing existing Cartographer nodes (if any)...")
        try:
            subprocess.run(['pkill', '-f', 'cartographer_ros.*cartographer_node'], check=False)
            subprocess.run(['pkill', '-f', 'cartographer_ros.*cartographer_occupancy_grid_node'], check=False)
        except Exception as e:
            self.get_logger().warn(f'Failed to kill Cartographer nodes (may be harmless): {e}')

        # Give ROS time to clean up
        time.sleep(2)

        # Get the config directory for Cartographer and restart the nodes
        try:
            config_dir = subprocess.check_output(
                "source /opt/ros/humble/setup.bash && source ~/carto_ws/install/setup.bash && ros2 pkg prefix ros2_mapping_support",
                shell=True, stderr=subprocess.PIPE
            ).decode('utf-8').strip() + "/share/ros2_mapping_support/config"
            self.get_logger().info(f"[reset_cartographer] Using config dir: {config_dir}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to get config directory: {e}")
            self.get_logger().error(f"stderr: {e.stderr.decode('utf-8')}")
            return

        self.get_logger().info("[reset_cartographer] Restarting cartographer_node...")
        try:
            # Start the Cartographer node
            subprocess.Popen(
                ['bash', '-lc', f"ros2 run cartographer_ros cartographer_node -configuration_directory {config_dir} -configuration_basename cartographer_2d_no_odom.lua --ros-args -r __node:=cartographer_node -r scan:=scan -r imu:=imu/data"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("[reset_cartographer] cartographer_node restarted successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restart cartographer_node: {e}")

        time.sleep(2)

        self.get_logger().info("[reset_cartographer] Restarting occupancy_grid_node...")
        try:
            # Start the occupancy grid node (map publisher)
            subprocess.Popen(
                ['bash', '-lc', "ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -r __node:=cartographer_occupancy_grid_node -p resolution:=0.05 -p publish_period_sec:=0.3"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("[reset_cartographer] occupancy_grid_node restarted successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restart occupancy_grid_node: {e}")

        self.get_logger().info("[reset_cartographer] Cartographer reset complete, new map started.")


def main(args=None):
    rclpy.init(args=args)
    node = TabletResetRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


====================
File: Readme.txt
====================
To Run download the carto_ws zip and extract on desktop.

After install the following repositories





#Terminal 1

pkill -f nav2_bringup || true
pkill -f controller_server || true
pkill -f planner_server || true
pkill -f bt_navigator || true
pkill -f amcl || true
pkill -f map_server || true

#Terminal 2

cd ~/carto_ws
colcon build --symlink-install
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_mapping_support cartographer_lidar_imu.launch.py \
  lidar_port:=/dev/ttyUSB0 i2c_bus:=7 i2c_address:=0x28

#Terminal 3

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo map base_link

#Terminal 4

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False autostart:=True slam:=False use_composition:=False \
  params_file:=/home/team4/carto_ws/src/ros2_mapping_support/config/nav2_params_cartographer_slam_resolved.yaml \
  map:=/home/team4/carto_ws/src/ros2_mapping_support/config/my_map.yaml

#Terminal 5

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ros2_mapping_support motor_driver_pca_reg_dual --ros-args \
  -p ena_addr:="'0x41'" -p enb_addr:="'0x60'" \
  -p ena_channel:=0 -p in1_channel:=1 -p in2_channel:=2 \
  -p enb_channel:=0 -p in3_channel:=1 -p in4_channel:=2 \
  -p pwm_freq_hz:=1000.0 \
  -p max_lin:=0.8 -p max_ang_cmd:=1.2 \
  -p deadband:=0.03 -p min_duty_pct:=80.0 \
  -p brake_on_zero:=false \
  -p invert_right:=true -p invert_left:=false \
  -p map_enA_to_left:=true \
  -p i2c_bus:=1 \
  -p cmd_topic:=/cmd_vel

#Terminal 6

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091

cd ~/carto_ws
export ROS_DOMAIN_ID=88
export FASTDDS_TRANSPORT_SHARED_MEM=off
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tablet_nav2.py



====================
File: TabletPather.py
====================
#!/usr/bin/env python3
#
# Listens to /tablet_goal (PoseStamped from the tablet UI)
# and forwards it to Nav2's NavigateToPose action.
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
w
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TabletGoalToNav2(Node):
    def __init__(self):
        super().__init__('tablet_goal_to_nav2')

        # Action client for Nav2
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to the goal published by the tablet UI
        self._tablet_goal_sub = self.create_subscription(
            PoseStamped,
            '/tablet_goal',         # must match HTML JS topic name
            self.tablet_goal_cb,
            10
        )

        self._current_goal_handle = None
        self._sending_goal = False

        self.get_logger().info('TabletGoalToNav2 node started. Waiting for /tablet_goal...')

    # Called whenever the tablet publishes a new PoseStamped
    def tablet_goal_cb(self, msg: PoseStamped):
        # If we’re already processing a goal, you can either cancel it or queue the new one.
        if self._sending_goal:
            self.get_logger().warn('Got new /tablet_goal while one is active. Cancelling old goal and sending new one.')
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()

        self._sending_goal = True

        # Ensure frame_id is valid; the HTML sets it to 'map'
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        self.get_logger().info(
            f'Received tablet goal: frame={msg.header.frame_id}, '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

        # Wait for the Nav2 action server
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available! Is nav2_bringup running?')
            self._sending_goal = False
            return

        # Wrap PoseStamped in NavigateToPose.Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.get_logger().info('Sending goal to Nav2...')
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

    # Called when the action server accepts/rejects the goal
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            self._sending_goal = False
            self._current_goal_handle = None
            return

        self.get_logger().info('Nav2 goal accepted.')
        self._current_goal_handle = goal_handle

        # Now wait for the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._nav_result_cb)

    # Optional feedback from Nav2 (current pose, distance remaining, etc.)
    def nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        # distance_remaining is in meters
        self.get_logger().debug(
            f'Nav2 feedback: distance_remaining={fb.distance_remaining:.2f} m'
        )

    # Called when Nav2 finishes (succeeded, aborted, canceled)
    def _nav_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        self.get_logger().info(f'Nav2 result received. Status={status}')
        # You can inspect result if needed (e.g., result.error_code).

        self._sending_goal = False
        self._current_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = TabletGoalToNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


====================
File: Tablet_nav2
====================
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TabletGoalToNav2(Node):
    def __init__(self):
        super().__init__('tablet_goal_to_nav2')

        # Subscribe to the tablet UI goal topic (unchanged)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/tablet_goal',
            self.tablet_goal_callback,
            10
        )

        # Nav2 action client (default server name)
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self._active_goal_handle = None
        self.get_logger().info(
            'TabletGoalToNav2 node started; listening on /tablet_goal'
        )

    def tablet_goal_callback(self, msg: PoseStamped):
        """
        Called whenever the tablet publishes a new goal.
        """
        self.get_logger().info(
            f'Received tablet goal: frame={msg.header.frame_id or "?"}, '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )

        # Make sure Nav2 is up
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('navigate_to_pose action server not available yet')
            return

        # Cancel previous Nav2 goal if one is active
        if self._active_goal_handle is not None:
            self.get_logger().info('Cancelling previous Nav2 goal')
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Failed to cancel previous goal: {e}')

        # Build Nav2 NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        # Stamp with current time (frame_id should already be 'map' from tablet)
        now = self.get_clock().now().to_msg()
        goal_msg.pose.header.stamp = now

        self.get_logger().info('Sending goal to Nav2 /navigate_to_pose')
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Called when Nav2 accepts/rejects the goal.
        """
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Error sending goal to Nav2: {e}')
            self._active_goal_handle = None
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self._active_goal_handle = None
            return

        self.get_logger().info('Nav2 goal accepted')
        self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Nav2 feedback (optional logging only).
        """
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'Nav2 feedback: distance_remaining={fb.distance_remaining:.3f}'
        )

    def result_callback(self, future):
        """
        Called when Nav2 finishes the goal.
        """
        try:
            result = future.result().result
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Error getting Nav2 result: {e}')
            return

        self.get_logger().info(f'Nav2 goal finished with status={status}')
        self._active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = TabletGoalToNav2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


====================
File: new_implemented_ros_state_machine.py
====================

#!/usr/bin/env python3
import os, time, collections, cv2, numpy as np,smbus
from ultralytics import YOLO
import Jetson.GPIO as GPIO
from Focuser import Focuser  # assuming you saved your previous class
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


# ==== USER SETTINGS ====
BUS_NUM      = 1
ADDR_ENA     = 0x41
ADDR_ENB     = 0x60

ENA_CH       = 0
IN1_CH       = 1
IN2_CH       = 2
ENB_CH       = 0
IN3_CH       = 1
IN4_CH       = 2

PWM_FREQ_HZ  = 1000.0
PWM_FREQ_HZ_SERVO = 50.0
MIN_DUTY     = 0.00
MAX_DUTY     = 1.00

INVERT_RIGHT = True
INVERT_LEFT  = False
# ========================

_MODE1      = 0x00
_MODE2      = 0x01
_LED0_ON_L  = 0x06
_PRESCALE   = 0xFE

_RESTART = 1 << 7
_SLEEP   = 1 << 4
_AI      = 1 << 5
_OUTDRV  = 1 << 2


#Clip high/low values
def _clip(v, lo, hi):
    return hi if v > hi else lo if v < lo else v

#Get Duty cycle from angle
def angle_to_duty(angle, freq_hz=50.0):
    MIN_US = 500
    MAX_US = 2500
    angle = max(0, min(180, angle))
    us = MIN_US + (MAX_US - MIN_US) * angle / 180.0
    period_us = 1_000_000.0 / freq_hz
    duty = us / period_us    # fraction of total period
    return duty              # e.g. 0.075 for 7.5%

# ======================
# PCA9685 CLASS
# ======================
class PCA9685:
    def __init__(self, bus, addr, freq_hz=300.0):
        self.bus = bus
        self.addr = addr
        self._w8(_MODE1, _SLEEP | _AI)
        self._w8(_MODE2, _OUTDRV)
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old & ~_SLEEP) | _RESTART | _AI)
        time.sleep(0.005)

    def set_pwm_freq(self, f_hz):
        prescale = int(round(25_000_000.0 / (4096.0 * float(f_hz)) - 1.0))
        prescale = _clip(prescale, 3, 255)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old | _SLEEP) & 0x7F)
        self._w8(_PRESCALE, prescale)
        self._w8(_MODE1, old | _AI)
        time.sleep(0.005)
        self._w8(_MODE1, old | _AI | _RESTART)

    def set_off(self, ch):
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0,0,0x10])

    def set_on(self, ch):
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0x10,0,0])

    def set_pwm(self, ch, duty01):
        duty01 = _clip(duty01, 0.0, 1.0)
        if duty01 <= 0.0: self.set_off(ch); return
        if duty01 >= 1.0: self.set_on(ch);  return
        off = int(round(duty01 * 4095.0))
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0, off & 0xFF, (off>>8)&0x0F])

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _r8(self, reg):
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

# ======================
# MOTOR DRIVER CLASS
#Initializes the right and lefft motor drivers
# ======================
class MotorDriver:
    def __init__(self):
        self.bus = smbus.SMBus(BUS_NUM)
        self.pcaA = PCA9685(self.bus, ADDR_ENA, PWM_FREQ_HZ)
        self.pcaB = PCA9685(self.bus, ADDR_ENB, PWM_FREQ_HZ)

        self.invR = INVERT_RIGHT
        self.invL = INVERT_LEFT

        self.stop()

    # ===== Direction Helpers =====
    def _right_forward(self):
        if not self.invR:
            self.pcaA.set_on(IN1_CH); self.pcaA.set_off(IN2_CH)
        else:
            self.pcaA.set_off(IN1_CH); self.pcaA.set_on(IN2_CH)

    def _right_reverse(self):
        if not self.invR:
            self.pcaA.set_off(IN1_CH); self.pcaA.set_on(IN2_CH)
        else:
            self.pcaA.set_on(IN1_CH); self.pcaA.set_off(IN2_CH)

    def _left_forward(self):
        if not self.invL:
            self.pcaB.set_on(IN3_CH); self.pcaB.set_off(IN4_CH)
        else:
            self.pcaB.set_off(IN3_CH); self.pcaB.set_on(IN4_CH)

    def _left_reverse(self):
        if not self.invL:
            self.pcaB.set_off(IN3_CH); self.pcaB.set_on(IN4_CH)
        else:
            self.pcaB.set_on(IN3_CH); self.pcaB.set_off(IN4_CH)

    # ===== Motor Enable =====
    def _enable_both(self, duty):
        duty = _clip(duty, 0.0, 1.0)
        if duty == 0:
            self.pcaA.set_off(ENA_CH)
            self.pcaB.set_off(ENB_CH)
        else:
            self.pcaA.set_pwm(ENA_CH, duty)
            self.pcaB.set_pwm(ENB_CH, duty)

    # ===== PUBLIC =====
    def stop(self):
        self.pcaA.set_off(IN1_CH); self.pcaA.set_off(IN2_CH)
        self.pcaB.set_off(IN3_CH); self.pcaB.set_off(IN4_CH)
        self._enable_both(0)

    def brake(self):
        self.pcaA.set_on(IN1_CH); self.pcaA.set_on(IN2_CH)
        self.pcaB.set_on(IN3_CH); self.pcaB.set_on(IN4_CH)
        self._enable_both(0)

    def tank_drive(self, left, right):
        """
        left, right ∈ [-1, 1]
        sign determines direction
        0 => that side is OFF (no drive)
        """
        # Clip commands
        left  = _clip(left,  -1, 1)
        right = _clip(right, -1, 1)

        # ----- LEFT SIDE -----
        if abs(left) < 1e-3:
            # turn LEFT motor fully off (both inputs low)
            left_duty = 0.0
            self.pcaB.set_off(IN3_CH)
            self.pcaB.set_off(IN4_CH)
        else:
            left_sign  = 1 if left > 0 else -1
            left_duty  = _clip(abs(left), MIN_DUTY, MAX_DUTY)
            if left_sign > 0:
                self._left_forward()
            else:
                self._left_reverse()

        # ----- RIGHT SIDE -----
        if abs(right) < 1e-3:
            # turn RIGHT motor fully off (both inputs low)
            right_duty = 0.0
            self.pcaA.set_off(IN1_CH)
            self.pcaA.set_off(IN2_CH)
        else:
            right_sign = 1 if right > 0 else -1
            right_duty = _clip(abs(right), MIN_DUTY, MAX_DUTY)
            if right_sign > 0:
                self._right_forward()
            else:
                self._right_reverse()

        # Enable both sides with the higher duty
        self._enable_both(max(left_duty, right_duty))

    def arcade_drive(self, v, w):
        left  = _clip(v + w, -1, 1)
        right = _clip(v - w, -1, 1)
        self.tank_drive(left, right)

    # ===== Simple Motion Helpers (for WASD) =====
    def forward(self):
        self.tank_drive(-1, -1)

    def reverse(self):
        self.tank_drive(1, 1)

    def turn_left(self):
        self.tank_drive(-1, 0)

    def turn_right(self):
        self.tank_drive(0, -1)

    def rotate_right(self):
        self.tank_drive(0.8, -0.8)

    def rotate_left(self):
        self.tank_drive(-0.8, 0.8)

    def shutdown(self):
        try:
            self.stop()
            self.bus.close()
        except:
            pass


class VisionEventPublisher(Node):
    """
    Lightweight publisher that lets the vision script raise events for a
    separate command/bridge node (which can talk to Nav2).
    """
    STOP_EVENT = 1

    def __init__(self):
        super().__init__("vision_event_publisher")
        self.event_pub = self.create_publisher(UInt8, "/vision/events", 10)

    def send_stop(self, reason=None):
        self.event_pub.publish(UInt8(data=self.STOP_EVENT))


def publish_stop(event_pub, reason=None):
    if event_pub is None:
        return
    event_pub.send_stop(reason)


#Gives control of motors if not active
def sit_down_bitch_control(key, active, motors):
    """
    Handle WASD motor control only if `active` is True.

    key: single-character string from input
    active: bool, whether motors should respond
    motors: object with methods to control motors (e.g., motors.forward(), motors.stop())
    """
    if not active:
        motors.stop()  # immediately stop motors when inactive
        pass
    else:
        # Only respond when active
        if key in ('w', 'W'):
            motors.forward()
        elif key in ('s', 'S'):
            motors.reverse()
        elif key in ('a', 'A'):
            motors.turn_left()
        elif key in ('d', 'D'):
            motors.turn_right()
        elif key in ('o', 'O'):
            motors.rotate_right()
        elif key in ('i', 'i'):
            motors.rotate_left()
        else:
            motors.stop()

#Initializes the servo motors
i2c_bus = smbus.SMBus(BUS_NUM)   # BUS_NUM is 1 in your settings

# Now create the servo driver using that bus
servo = PCA9685(bus=i2c_bus, addr=0x42, freq_hz=PWM_FREQ_HZ_SERVO)

# ---------- Jetson Nano Camera GStreamer Pipeline ----------
GST = (
    "nvarguscamerasrc sensor-id=0 !"
    "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 !"
    "nvvidconv !"
    "video/x-raw,format=BGRx !"
    "videoconvert ! video/x-raw,format=BGR !"
    "appsink drop=true max-buffers=4 sync=false"
)

SERVO_PIN = 22

GPIO.setmode(GPIO.BOARD)            # Sets up Laser GPIO and sets output to LOW
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(SERVO_PIN, GPIO.LOW)

# -------- Model config --------
ENGINE_PATH = "/home/team4/Camera/green_specific.engine"   # your TensorRT engine
DEVICE      = 0
IMGSZ       = 704
CONF        = 0.25
HALF        = True

# -------- Load engine (explicit task to silence warning) --------
model = YOLO(ENGINE_PATH, task="detect")

# -------- Camera --------
cap = cv2.VideoCapture(GST, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise SystemExit("Failed to open camera.")

WIN = "IMX519 YOLOv8 TensorRT"
cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

times = collections.deque(maxlen=60)
prev  = time.time()

# ---------- State variables ----------
focuser = Focuser(bus=10)

# State 1 variables
state = 1
print("STATE INIT -> 1")
focus_pos = 300
conf_max = 0.0
servo_offset = 30        #Adjust servo offset if needed
acquiring = True
object_detected = False  #Object detection fields
angle = 30
batch_conf = []

# State 2 variables
focus_positions = list(range(650, 0, -50))  # [750, 700, ..., 50]
current_index = 0
ideal_focus = 300
object_detected_once = False

# State 3 Variables
acquire_tol = 150          # px: stay in sweep until |error| <= this
track_tol   = 10           # px: no movement if within this band (deadband)
direction = 1
focus_hits_state8 = 0

# State 4 Variables
past_conf = 0.0
error = .30
A = 5  # focus step size (change if you want faster/slower focus sweeps)

# State 5 Variables
CENTER_MIN = 0
CENTER_MAX = 180
px_cushion   = 30          # cushion in pixels around image center
slow_step    = 0.5         # degrees per nudge to keep motion slow

# State 6 Variables
laser_toggle_count = 0

#State 9 Variables
object_servo_angle = 0   # record servo angle when entering state 9


#Intialize motors for manuel control
rclpy.init(args=None)
event_pub = VisionEventPublisher()
motors = MotorDriver()
active = True
key = ''   # no key pressed yet
sit_down_bitch_control(key, active, motors)

#Sets initial servo position
duty = angle_to_duty(angle)
servo.set_pwm(0, duty)

time.sleep(1)


try:
    while True:
        if((state == 1)|(state == 7)):
            sit_down_bitch_control(key, active, motors)

        ok, frame_bgr = cap.read()
        if not ok:
            break

        results = model.predict(
            source=frame_bgr,
            device=DEVICE,
            imgsz=IMGSZ,
            conf=CONF,
            half=HALF,
            verbose=False
        )

        frame_width = frame_bgr.shape[1]
        center_x = frame_width / 2
        tolerance = 20

        r = results[0]

        current_time = time.time()

        if (0.1 <= (current_time - prev) < 0.25):
            if len(r.boxes.conf) > 0:
                batch_conf.append(float(r.boxes.conf.max()))
            else:
                batch_conf.append(0.0)

        # top-conf x_center in pixels
        x_center = float(r.boxes.xywh[r.boxes.conf.argmax(), 0]) if len(r.boxes) > 0 else None

        # normalized center for TOY SOLDIER (prefer), else top-conf box
        x_center_n = None

        if len(r.boxes) > 0:
            try:
                names = r.names if hasattr(r, "names") else model.names
                cls_ids = r.boxes.cls.cpu().numpy().astype(int)
                xywhn   = r.boxes.xywhn.cpu().numpy()
                confs   = r.boxes.conf.cpu().numpy()

                # try to pick "toy soldier" if present
                target_idx = None
                if names is not None:
                    for i, cid in enumerate(cls_ids):
                        if str(names.get(int(cid), "")).lower() == "toy soldier":
                            if target_idx is None or confs[i] > confs[target_idx]:
                                target_idx = i
                # fallback to highest confidence
                if target_idx is None:
                    target_idx = int(np.argmax(confs))

                x_center_n = float(xywhn[target_idx, 0])  # normalized 0..1
            except Exception:
                if x_center is not None and frame_width > 0:
                    x_center_n = float(x_center / frame_width)

        annotated = results[0].plot()
        cv2.imshow(WIN, annotated)

        # IMPORTANT: let OpenCV process GUI events
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        # Update `key` only when something is actually pressed
        if k != 255:              # 255 (or -1 before &0xFF) means "no key"
            key = chr(k)          # convert int → 'w', 'a', 's', 'd', etc.


        # ---------- Per-frame state logic ----------
        if state == 1:
            sit_down_bitch_control(key, active, motors)
            acquiring = True
            if angle > 130:
                angle = 130
                direction = -1
            elif angle < 30:
                angle = 30
                direction = 1

            # sweep
            angle += 1 * direction

            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

            if object_detected:
                focus_hits_state8 = 0
                print("STATE -> 8")
                state = 8
                active = False
                sit_down_bitch_control(key, active, motors)
                motors.stop()
                publish_stop(event_pub, "state1->8_detected")


        if state == 3:
            if x_center is not None:
                error_pixels = center_x - x_center  # + => target left of center

                if acquiring and abs(error_pixels) > acquire_tol:
                    # keep sweeping in the current direction
                    angle += 1 * direction
                else:
                    acquiring = False
                    if abs(error_pixels) > track_tol:
                        state = 4  # switch to fine-tracking state
                        print("STATE -> 4")
            else:
                acquiring = True
                angle += 1 * direction

            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

        if state == 5:
            if x_center_n is not None and frame_width > 0:
                margin_n = float(px_cushion) / float(frame_width)
                # nudge slowly toward the center
                if x_center_n < (0.5 - margin_n):
                    angle += slow_step     # target is left in image → turn left
                elif x_center_n > (0.5 + margin_n):
                    angle -= slow_step     # target is right in image → turn right
                # clamp to mechanical limits
                if angle > CENTER_MAX:
                    angle = CENTER_MAX
                elif angle < CENTER_MIN:
                    angle = CENTER_MIN
            # if no detection, just hold current angle
            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

        if state == 9:
            print(state)
            # lock servo to 110 deg and sync logical angle
            angle = 110 - servo_offset
            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

            # --- motor correction runs together with servo motion ---
            if x_center is not None:
                error_pixels = center_x - x_center  # + => target left of center

                if acquiring and abs(error_pixels) > acquire_tol:
                    # sweeping: rotate in the sweep direction
                    if error_pixels > acquire_tol:
                        # target is left in the image → rotate to reduce error
                        motors.rotate_left()
                    elif error_pixels < -acquire_tol:
                        #target is right in the image
                        motors.rotate_right()

                else:
                    acquiring = False

                    # Fine tracking threshold → go to state 4
                    if abs(error_pixels) > track_tol:
                        # Rotate toward error BEFORE switching
                        if error_pixels > 0:
                            print("3")
                            motors.rotate_right()
                        else:
                            print("4")
                            motors.rotate_left()

                            state = 10
                            print("STATE -> 10")
                    else:
                        motors.stop()  # aligned with target
                        publish_stop(event_pub, "state9_aligned")
            else:
                # No target: sweep
                acquiring = True
                if object_servo_angle > 110:
                    motors.rotate_left()
                elif object_servo_angle < 110:
                    motors.rotate_right()


        if state == 10:
            motors.forward()
            boxes = r.boxes  # all detected boxes

            if boxes is not None and len(boxes) > 0:
                # take the biggest object
                biggest = max(boxes, key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) *
                                      (b.xyxy[0][3] - b.xyxy[0][1]))

                x1, y1, x2, y2 = biggest.xyxy[0]  # tensor → floats

                box_area = (x2 - x1) * (y2 - y1)
                frame_area = 704 * 704
                fill_ratio = box_area / frame_area   # value between 0 and 1

                # WHERE THE ROBOT DRIFTS → send back to STATE 8
                error_pixels = center_x - ((x1 + x2) / 2)

                # >>> ONLY THIS LINE CHANGED <<<
                if abs(error_pixels) > track_tol * 20:   # was * 2
                    motors.stop()
                    publish_stop(event_pub, "state10_reacquire")
                    state = 8  # ← return to reacquire alignment
                    print("STATE -> 8")
                    continue

                # Close enough
                if fill_ratio >= 0.30:
                    motors.stop()
                    publish_stop(event_pub, "state10_target_close")
                    state = 2
                    print("STATE -> 2")


        # ---------- 0.25 s logic ----------
        if current_time - prev >= .25:
            batch_max_conf = np.mean(batch_conf) if batch_conf else 0.0
            if np.isnan(batch_max_conf) or batch_max_conf < 0.5:
                batch_max_conf = 0.0

            if state == 1:
                if batch_max_conf >= 0.75:
                    focus_pos = 750
                    object_detected = True
                    acquiring = True
                    print(f"Object detected with confidence {batch_max_conf:.2f}, switching to state 1→2 for focusing.")
                    current_index = 0
                    conf_max = 0.0
                    ideal_focus = focus_pos

            if state == 2:  # Autofocus sweep state
                if batch_max_conf >= conf_max:
                    conf_max = batch_max_conf
                    ideal_focus = focus_pos
                    print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                # move to next focus position safely
                current_index += 1
                if current_index >= len(focus_positions):
                    # finished full sweepq
                    if conf_max < 0.5:
                        object_detected_once = False
                        focus_pos = 300
                        current_index = 0
                        state = 1
                        print("STATE -> 1")
                        print("Sweep done, low confidence. Returning to scan.")
                    else:
                        object_detected_once = False
                        focus_pos = ideal_focus
                        print(
                            f"Completed first full sweep after detection. "
                            f"Best focus: {ideal_focus} with conf: {conf_max}"
                        )
                        current_index = 0
                        state = 3
                        print("STATE -> 3")
                else:
                    focus_pos = focus_positions[current_index]


            if state == 4:
                print(state)
                focus_pos -= A
                if batch_max_conf <= past_conf - 0.01:
                    A = -A
                    print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")
                if batch_max_conf < (past_conf - (past_conf * error)):
                    state = 1
                    print("STATE -> 1")
                    focus_pos = 300
                    conf_max = 0
                    ideal_focus = 0
                    print(f"Cannot refine focus, returning to scan. Last good conf: {past_conf}")
                if batch_max_conf >= past_conf and batch_max_conf >= 0.5:
                    state = 5
                    print("STATE -> 5")
                    print(f"Object in focus with sufficient confidence. Holding focus at: {focus_pos} with conf: {batch_max_conf}")

            if state == 5:
                # dwell here some time before firing laser
                laser_toggle_count += 1
                if laser_toggle_count > 16:  # ~4 s at 0.25s per tick
                    laser_toggle_count = 0
                    state = 6
                    print("STATE -> 6")

            if state == 6:
                # LASER ON window
                if laser_toggle_count == 0:
                    GPIO.output(SERVO_PIN, GPIO.HIGH)  # turn laser ON once
                laser_toggle_count += 1
                print(6)
                if laser_toggle_count >= 40:  # ~10 s ON time
                    GPIO.output(SERVO_PIN, GPIO.LOW)   # turn laser OFF
                    laser_toggle_count = 0
                    state = 7
                    print("STATE -> 7")
                    active = True
                    sit_down_bitch_control(key, active, motors)

            if state == 7:
                # cool-down / re-enable motors, keep laser OFF
                laser_toggle_count += 1
                if laser_toggle_count >= 40:  # ~10 s cool-downo
                    focus_pos = 300
                    laser_toggle_count = 0
                    GPIO.output(SERVO_PIN, GPIO.LOW)   # ensure laser stays OFF
                    state = 1
                    print("STATE -> 1")
                    conf_max = 0.0
                    acquiring = True
                    object_detected = False
                    angle = 30

            if state == 8:  # Autofocus sweep state
                if batch_max_conf >= conf_max:
                    conf_max = batch_max_conf
                    ideal_focus = focus_pos
                    print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                current_index += 1
                if current_index >= len(focus_positions):
                    # finished full sweep
                    if ((conf_max < 0.5)|(focus_hits_state8 >=4)):
                        object_detected = False        # <<< ADD THIS
                        object_detected_once = False
                        focus_pos = 300
                        current_index = 0
                        focus_hits_state8 = 0
                        conf_max = 0.0                 # optional but nice reset
                        active = True
                        sit_down_bitch_control(key, active, motors)
                        state = 1
                        print("STATE -> 1")
                        print("Sweep done, low confidence. Returning to scan.")
                    else:
                        object_detected_once = False
                        focus_pos = ideal_focus
                        ideal_focus = 0
                        acquiring = True
                        print("Completed first full sweep after detection. "
                        f"Best focus: {ideal_focus} with conf: {conf_max}")
                        current_index = 0

                        # store servo angle when moving to state 9
                        object_servo_angle = angle

                        state = 9
                        print("STATE -> 9")
                        servo_reached_9 = False

                else:
                    focus_pos = focus_positions[current_index]

                if(batch_max_conf <.7):
                    focus_hits_state8 += 1


            if state == 10:
                focus_pos -= A
                if batch_max_conf <= past_conf - 0.01:
                    A = -A
                    print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")

            # update past_conf for next tick (used in state 4)

            focuser.set(Focuser.OPT_FOCUS, focus_pos)
            past_conf = batch_max_conf
            prev = current_time
            batch_conf.clear()

finally:
    cap.release()
    cv2.destroyAllWindows()
    try:
        publish_stop(event_pub, "shutdown")
        event_pub.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


====================
File: new_implemented_ros_state_machine_v2.py
====================

#!/usr/bin/env python3
import os, time, collections, threading, cv2, numpy as np, smbus
from ultralytics import YOLO
import Jetson.GPIO as GPIO
from Focuser import Focuser  # assuming you saved your previous class
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import LaserScan
import tf2_ros
from rclpy.duration import Duration


# ==== USER SETTINGS ====
BUS_NUM      = 1
ADDR_ENA     = 0x41
ADDR_ENB     = 0x60

ENA_CH       = 0
IN1_CH       = 1
IN2_CH       = 2
ENB_CH       = 0
IN3_CH       = 1
IN4_CH       = 2

PWM_FREQ_HZ  = 1000.0
PWM_FREQ_HZ_SERVO = 50.0
MIN_DUTY     = 0.00
MAX_DUTY     = 1.00

INVERT_RIGHT = True
INVERT_LEFT  = False
# ========================

_MODE1      = 0x00
_MODE2      = 0x01
_LED0_ON_L  = 0x06
_PRESCALE   = 0xFE

_RESTART = 1 << 7
_SLEEP   = 1 << 4
_AI      = 1 << 5
_OUTDRV  = 1 << 2


#Clip high/low values
def _clip(v, lo, hi):
    return hi if v > hi else lo if v < lo else v

#Get Duty cycle from angle
def angle_to_duty(angle, freq_hz=50.0):
    MIN_US = 500
    MAX_US = 2500
    angle = max(0, min(180, angle))
    us = MIN_US + (MAX_US - MIN_US) * angle / 180.0
    period_us = 1_000_000.0 / freq_hz
    duty = us / period_us    # fraction of total period
    return duty              # e.g. 0.075 for 7.5%

def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

# ======================
# PCA9685 CLASS
# ======================
class PCA9685:
    def __init__(self, bus, addr, freq_hz=300.0):
        self.bus = bus
        self.addr = addr
        self._w8(_MODE1, _SLEEP | _AI)
        self._w8(_MODE2, _OUTDRV)
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old & ~_SLEEP) | _RESTART | _AI)
        time.sleep(0.005)

    def set_pwm_freq(self, f_hz):
        prescale = int(round(25_000_000.0 / (4096.0 * float(f_hz)) - 1.0))
        prescale = _clip(prescale, 3, 255)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old | _SLEEP) & 0x7F)
        self._w8(_PRESCALE, prescale)
        self._w8(_MODE1, old | _AI)
        time.sleep(0.005)
        self._w8(_MODE1, old | _AI | _RESTART)

    def set_off(self, ch):
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0,0,0x10])

    def set_on(self, ch):
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0x10,0,0])

    def set_pwm(self, ch, duty01):
        duty01 = _clip(duty01, 0.0, 1.0)
        if duty01 <= 0.0: self.set_off(ch); return
        if duty01 >= 1.0: self.set_on(ch);  return
        off = int(round(duty01 * 4095.0))
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0, off & 0xFF, (off>>8)&0x0F])

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _r8(self, reg):
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

# ======================
# MOTOR DRIVER CLASS
#Initializes the right and lefft motor drivers
# ======================
class MotorDriver:
    def __init__(self):
        self.bus = smbus.SMBus(BUS_NUM)
        self.pcaA = PCA9685(self.bus, ADDR_ENA, PWM_FREQ_HZ)
        self.pcaB = PCA9685(self.bus, ADDR_ENB, PWM_FREQ_HZ)

        self.invR = INVERT_RIGHT
        self.invL = INVERT_LEFT

        self.stop()

    # ===== Direction Helpers =====
    def _right_forward(self):
        if not self.invR:
            self.pcaA.set_on(IN1_CH); self.pcaA.set_off(IN2_CH)
        else:
            self.pcaA.set_off(IN1_CH); self.pcaA.set_on(IN2_CH)

    def _right_reverse(self):
        if not self.invR:
            self.pcaA.set_off(IN1_CH); self.pcaA.set_on(IN2_CH)
        else:
            self.pcaA.set_on(IN1_CH); self.pcaA.set_off(IN2_CH)

    def _left_forward(self):
        if not self.invL:
            self.pcaB.set_on(IN3_CH); self.pcaB.set_off(IN4_CH)
        else:
            self.pcaB.set_off(IN3_CH); self.pcaB.set_on(IN4_CH)

    def _left_reverse(self):
        if not self.invL:
            self.pcaB.set_off(IN3_CH); self.pcaB.set_on(IN4_CH)
        else:
            self.pcaB.set_on(IN3_CH); self.pcaB.set_off(IN4_CH)

    # ===== Motor Enable =====
    def _enable_both(self, duty):
        duty = _clip(duty, 0.0, 1.0)
        if duty == 0:
            self.pcaA.set_off(ENA_CH)
            self.pcaB.set_off(ENB_CH)
        else:
            self.pcaA.set_pwm(ENA_CH, duty)
            self.pcaB.set_pwm(ENB_CH, duty)

    # ===== PUBLIC =====
    def stop(self):
        self.pcaA.set_off(IN1_CH); self.pcaA.set_off(IN2_CH)
        self.pcaB.set_off(IN3_CH); self.pcaB.set_off(IN4_CH)
        self._enable_both(0)

    def brake(self):
        self.pcaA.set_on(IN1_CH); self.pcaA.set_on(IN2_CH)
        self.pcaB.set_on(IN3_CH); self.pcaB.set_on(IN4_CH)
        self._enable_both(0)

    def tank_drive(self, left, right):
        """
        left, right ∈ [-1, 1]
        sign determines direction
        0 => that side is OFF (no drive)
        """
        # Clip commands
        left  = _clip(left,  -1, 1)
        right = _clip(right, -1, 1)

        # ----- LEFT SIDE -----
        if abs(left) < 1e-3:
            # turn LEFT motor fully off (both inputs low)
            left_duty = 0.0
            self.pcaB.set_off(IN3_CH)
            self.pcaB.set_off(IN4_CH)
        else:
            left_sign  = 1 if left > 0 else -1
            left_duty  = _clip(abs(left), MIN_DUTY, MAX_DUTY)
            if left_sign > 0:
                self._left_forward()
            else:
                self._left_reverse()

        # ----- RIGHT SIDE -----
        if abs(right) < 1e-3:
            # turn RIGHT motor fully off (both inputs low)
            right_duty = 0.0
            self.pcaA.set_off(IN1_CH)
            self.pcaA.set_off(IN2_CH)
        else:
            right_sign = 1 if right > 0 else -1
            right_duty = _clip(abs(right), MIN_DUTY, MAX_DUTY)
            if right_sign > 0:
                self._right_forward()
            else:
                self._right_reverse()

        # Enable both sides with the higher duty
        self._enable_both(max(left_duty, right_duty))

    def arcade_drive(self, v, w):
        left  = _clip(v + w, -1, 1)
        right = _clip(v - w, -1, 1)
        self.tank_drive(left, right)

    # ===== Simple Motion Helpers (for WASD) =====
    def forward(self):
        self.tank_drive(-1, -1)

    def reverse(self):
        self.tank_drive(1, 1)

    def turn_left(self):
        self.tank_drive(-1, 0)

    def turn_right(self):
        self.tank_drive(0, -1)

    def rotate_right(self):
        self.tank_drive(0.8, -0.8)

    def rotate_left(self):
        self.tank_drive(-0.8, 0.8)

    def shutdown(self):
        try:
            self.stop()
            self.bus.close()
        except:
            pass


# ROS 2 / Nav2 integration to intercept goals, pause Nav2, run the manual
# alignment/laser sequence, and then resume the saved goal.
class Nav2Interrupter(Node):
    def __init__(self):
        super().__init__("nav2_manual_alignment_bridge_v2")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cancel_client = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.goal_subs = [
            self.create_subscription(PoseStamped, "goal_pose", self._goal_cb, 10),
            self.create_subscription(PoseStamped, "move_base_simple/goal", self._goal_cb, 10),
        ]
        self.scan_sub = self.create_subscription(LaserScan, "scan", self._scan_cb, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pending_goal = None
        self.sequence_active = False
        self.sequence_progressed = False
        self.reset_requested = False

        self.approach_in_progress = False
        self.approach_goal_handle = None
        self.approach_result_future = None
        self.approach_completed = False
        self.approach_failed = False
        self.approach_target = None
        self.approach_clearance = 0.5

        self.known_enemies = []
        self.enemy_diameter = 0.333
        self.enemy_radius = self.enemy_diameter / 2.0
        self.pending_enemy_pose = None

        self.scan_lock = threading.Lock()
        self.last_scan = None

    def _goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            "Captured navigation goal; canceling Nav2 and starting manual alignment sequence."
        )
        self.pending_goal = msg
        self.sequence_active = True
        self.sequence_progressed = False
        self.reset_requested = True
        self._reset_approach()
        self._cancel_nav2_goal()

    def _cancel_nav2_goal(self):
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "Nav2 cancel service unavailable; Nav2 may keep moving until the action server stops."
            )
            return
        req = CancelGoal.Request()
        self.cancel_client.call_async(req)

    def _scan_cb(self, msg: LaserScan):
        with self.scan_lock:
            self.last_scan = msg

    def get_front_range(self, window_deg: float = 15.0):
        with self.scan_lock:
            scan = self.last_scan
        if scan is None or scan.angle_increment == 0.0:
            return None
        half = math.radians(window_deg) / 2.0
        best = None
        for i, rng in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= half and math.isfinite(rng) and scan.range_min < rng < scan.range_max:
                best = rng if best is None else min(best, rng)
        return best

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return None
        trans = tf.transform.translation
        rot = tf.transform.rotation
        yaw = quat_to_yaw(rot.x, rot.y, rot.z, rot.w)
        return (trans.x, trans.y, yaw)

    def estimate_enemy_pose(self, front_range=None):
        pose = self.get_robot_pose()
        if pose is None:
            return None
        if front_range is None:
            front_range = self.get_front_range()
        if front_range is None or not math.isfinite(front_range):
            front_range = 1.0
        x, y, yaw = pose
        enemy_x = x + math.cos(yaw) * front_range
        enemy_y = y + math.sin(yaw) * front_range
        return (enemy_x, enemy_y)

    def enemy_already_known(self, enemy_pose, margin: float | None = None):
        if enemy_pose is None:
            return False
        if margin is None:
            margin = self.enemy_radius
        ex, ey = enemy_pose
        for kx, ky in self.known_enemies:
            if math.hypot(ex - kx, ey - ky) <= (self.enemy_radius + margin):
                return True
        return False

    def record_enemy(self, enemy_pose):
        if enemy_pose is None:
            return
        self.known_enemies.append(enemy_pose)
        self.get_logger().info(f"Recorded enemy at {enemy_pose} with radius {self.enemy_radius:.3f} m.")

    def start_approach(self, front_range=None):
        if self.approach_in_progress or self.approach_completed:
            return
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Cannot start approach: no robot pose.")
            return
        if front_range is None:
            front_range = self.get_front_range()
        if front_range is None or not math.isfinite(front_range):
            front_range = 1.0
        x, y, yaw = robot_pose
        goal_dist = max(front_range - self.approach_clearance, 0.2)
        target_x = x + math.cos(yaw) * goal_dist
        target_y = y + math.sin(yaw) * goal_dist
        self.approach_target = (target_x, target_y, yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 action server not ready; cannot approach target.")
            return
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._approach_goal_response)
        self.get_logger().info(
            f"Sent approach goal to ({target_x:.2f}, {target_y:.2f}) "
            f"with clearance {self.approach_clearance:.2f} m (front range: {front_range:.2f})."
        )

    def _approach_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to send approach goal: {exc}")
            self.approach_failed = True
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the approach goal.")
            self.approach_failed = True
            return

        self.approach_goal_handle = goal_handle
        self.approach_in_progress = True
        self.approach_result_future = goal_handle.get_result_async()
        self.approach_result_future.add_done_callback(self._approach_result_cb)

    def _approach_result_cb(self, future):
        self.approach_in_progress = False
        try:
            _ = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Approach goal failed: {exc}")
            self.approach_failed = True
            return
        self.approach_completed = True
        self.get_logger().info("Approach goal completed; within desired stand-off distance.")

    def cancel_approach(self):
        if self.approach_goal_handle is not None:
            try:
                self.approach_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._reset_approach()

    def _reset_approach(self):
        self.approach_in_progress = False
        self.approach_goal_handle = None
        self.approach_result_future = None
        self.approach_completed = False
        self.approach_failed = False
        self.approach_target = None
        self.pending_enemy_pose = None

    def mark_sequence_complete(self):
        self.sequence_active = False
        self.sequence_progressed = False
        self.reset_requested = False
        self._reset_approach()
        self._resume_nav()

    def _resume_nav(self):
        if self.pending_goal is None:
            self.get_logger().warn("No stored goal to resume.")
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                "Nav2 action server not ready; keeping goal queued until the next completion event."
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.pending_goal
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to send resume goal: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the resume goal.")
            return

        self.get_logger().info("Nav2 goal accepted; navigation resumed toward the original destination.")
        self.pending_goal = None


#Initializes the servo motors
i2c_bus = smbus.SMBus(BUS_NUM)   # BUS_NUM is 1 in your settings

# Now create the servo driver using that bus
servo = PCA9685(bus=i2c_bus, addr=0x42, freq_hz=PWM_FREQ_HZ_SERVO)

# ---------- Jetson Nano Camera GStreamer Pipeline ----------
GST = (
    "nvarguscamerasrc sensor-id=0 !"
    "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 !"
    "nvvidconv !"
    "video/x-raw,format=BGRx !"
    "videoconvert ! video/x-raw,format=BGR !"
    "appsink drop=true max-buffers=4 sync=false"
)

SERVO_PIN = 22

GPIO.setmode(GPIO.BOARD)            # Sets up Laser GPIO and sets output to LOW
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(SERVO_PIN, GPIO.LOW)

# -------- Model config --------
ENGINE_PATH = "green_specific.engine"   # your TensorRT engine
DEVICE      = 0
IMGSZ       = 704
CONF        = 0.25
HALF        = True

# -------- Load engine (explicit task to silence warning) --------
model = YOLO(ENGINE_PATH, task="detect")

# -------- Camera --------
cap = cv2.VideoCapture(GST, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise SystemExit("Failed to open camera.")

WIN = "IMX519 YOLOv8 TensorRT"
cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

times = collections.deque(maxlen=60)
prev  = time.time()

# ---------- State variables ----------
focuser = Focuser(bus=10)

# State 1 variables
state = 1
print("STATE INIT -> 1")
focus_pos = 300
conf_max = 0.0
servo_offset = 30        #Adjust servo offset if needed
acquiring = True
object_detected = False  #Object detection fields
angle = 30
batch_conf = []

# State 2 variables
focus_positions = list(range(650, 0, -50))  # [750, 700, ..., 50]
current_index = 0
ideal_focus = 300
object_detected_once = False

# State 3 Variables
acquire_tol = 150          # px: stay in sweep until |error| <= this
track_tol   = 10           # px: no movement if within this band (deadband)
direction = 1
SWEEP_STEP_DEG = 2.0       # degrees per sweep step (increase to pan faster)
SWEEP_MIN_ANGLE = 30
SWEEP_MAX_ANGLE = 130

# State 4 Variables
past_conf = 0.0
error = .30
A = 5  # focus step size (change if you want faster/slower focus sweeps)

# State 5 Variables
CENTER_MIN = 0
CENTER_MAX = 180
px_cushion   = 30          # cushion in pixels around image center
slow_step    = 0.5         # degrees per nudge to keep motion slow

# State 6 Variables
laser_toggle_count = 0

#State 9 Variables
object_servo_angle = 0   # record servo angle when entering state 9

approach_goal_started = False  # track Nav2 approach command


# Initialize motors for the manual alignment/approach sequence
motors = MotorDriver()

#Sets initial servo position
duty = angle_to_duty(angle)
servo.set_pwm(0, duty)

time.sleep(1)


nav_bridge = None
executor = None
spin_thread = None


try:
    rclpy.init(args=None)
    nav_bridge = Nav2Interrupter()
    executor = MultiThreadedExecutor()
    executor.add_node(nav_bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def reset_state_machine():
        global state, focus_pos, conf_max, acquiring, object_detected, angle
        global current_index, ideal_focus, object_detected_once, direction
        global past_conf, laser_toggle_count, object_servo_angle, prev
        global approach_goal_started
        state = 1
        focus_pos = 300
        conf_max = 0.0
        acquiring = True
        object_detected = False
        angle = SWEEP_MIN_ANGLE
        batch_conf.clear()
        current_index = 0
        ideal_focus = 300
        object_detected_once = False
        direction = 1
        past_conf = 0.0
        laser_toggle_count = 0
        object_servo_angle = 0
        prev = time.time()
        approach_goal_started = False
        if nav_bridge is not None:
            nav_bridge.cancel_approach()
            nav_bridge.pending_enemy_pose = None
        motors.stop()
        duty = angle_to_duty(angle + servo_offset)
        servo.set_pwm(0, duty)
        print("STATE -> 1 (reset for new Nav2 goal)")

    previous_state = state

    while True:
        if nav_bridge.reset_requested:
            reset_state_machine()
            nav_bridge.reset_requested = False

        ok, frame_bgr = cap.read()
        if not ok:
            break

        results = model.predict(
            source=frame_bgr,
            device=DEVICE,
            imgsz=IMGSZ,
            conf=CONF,
            half=HALF,
            verbose=False
        )

        frame_width = frame_bgr.shape[1]
        center_x = frame_width / 2
        tolerance = 20

        r = results[0]

        current_time = time.time()

        if (0.1 <= (current_time - prev) < 0.25):
            if len(r.boxes.conf) > 0:
                batch_conf.append(float(r.boxes.conf.max()))
            else:
                batch_conf.append(0.0)

        # top-conf x_center in pixels
        x_center = float(r.boxes.xywh[r.boxes.conf.argmax(), 0]) if len(r.boxes) > 0 else None

        # normalized center for TOY SOLDIER (prefer), else top-conf box
        x_center_n = None

        if len(r.boxes) > 0:
            try:
                names = r.names if hasattr(r, "names") else model.names
                cls_ids = r.boxes.cls.cpu().numpy().astype(int)
                xywhn   = r.boxes.xywhn.cpu().numpy()
                confs   = r.boxes.conf.cpu().numpy()

                # try to pick "toy soldier" if present
                target_idx = None
                if names is not None:
                    for i, cid in enumerate(cls_ids):
                        if str(names.get(int(cid), "")).lower() == "toy soldier":
                            if target_idx is None or confs[i] > confs[target_idx]:
                                target_idx = i
                # fallback to highest confidence
                if target_idx is None:
                    target_idx = int(np.argmax(confs))

                x_center_n = float(xywhn[target_idx, 0])  # normalized 0..1
            except Exception:
                if x_center is not None and frame_width > 0:
                    x_center_n = float(x_center / frame_width)

        annotated = results[0].plot()
        cv2.imshow(WIN, annotated)

        # IMPORTANT: let OpenCV process GUI events
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        # ---------- Per-frame state logic ----------
        if state == 1:
            acquiring = True
            if angle > SWEEP_MAX_ANGLE:
                angle = SWEEP_MAX_ANGLE
                direction = -1
            elif angle < SWEEP_MIN_ANGLE:
                angle = SWEEP_MIN_ANGLE
                direction = 1

            # sweep
            angle += SWEEP_STEP_DEG * direction

            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

            if object_detected:
                state = 8
                print("STATE -> 8")
                motors.stop()

        if state == 3:
            if x_center is not None:
                error_pixels = center_x - x_center  # + => target left of center

                if acquiring and abs(error_pixels) > acquire_tol:
                    # keep sweeping in the current direction
                    angle += SWEEP_STEP_DEG * direction
                else:
                    acquiring = False
                    if abs(error_pixels) > track_tol:
                        state = 4  # switch to fine-tracking state
                        print("STATE -> 4")
            else:
                acquiring = True
                angle += SWEEP_STEP_DEG * direction

            # clamp sweep range
            if angle > SWEEP_MAX_ANGLE:
                angle = SWEEP_MAX_ANGLE
                direction = -1
            elif angle < SWEEP_MIN_ANGLE:
                angle = SWEEP_MIN_ANGLE
                direction = 1

            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

        if state == 5:
            if x_center_n is not None and frame_width > 0:
                margin_n = float(px_cushion) / float(frame_width)
                # nudge slowly toward the center
                if x_center_n < (0.5 - margin_n):
                    angle += slow_step     # target is left in image → turn left
                elif x_center_n > (0.5 + margin_n):
                    angle -= slow_step     # target is right in image → turn right
                # clamp to mechanical limits
                if angle > CENTER_MAX:
                    angle = CENTER_MAX
                elif angle < CENTER_MIN:
                    angle = CENTER_MIN
            # if no detection, just hold current angle
            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

        if state == 9:
            print(state)
            # lock servo to 110 deg and sync logical angle
            angle = 110 - servo_offset
            duty = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty)

            # Start Nav2 approach once when centered
            if nav_bridge and nav_bridge.sequence_active and not approach_goal_started:
                nav_bridge.start_approach(nav_bridge.get_front_range())
                approach_goal_started = True

            # Keep servo roughly aligned without driving the chassis (Nav2 drives)
            if x_center is not None:
                error_pixels = center_x - x_center
                if abs(error_pixels) > track_tol:
                    delta = slow_step if error_pixels > 0 else -slow_step
                    angle = max(CENTER_MIN, min(CENTER_MAX, angle + delta))
                    servo.set_pwm(0, angle_to_duty(angle + servo_offset))
            motors.stop()

            if nav_bridge and nav_bridge.sequence_active and (nav_bridge.approach_completed or nav_bridge.approach_failed):
                if nav_bridge.pending_enemy_pose is None:
                    nav_bridge.pending_enemy_pose = nav_bridge.estimate_enemy_pose(nav_bridge.get_front_range())
                state = 5
                laser_toggle_count = 0
                print("STATE -> 5 (Nav2 approach finished)")

        if state == 10:
            motors.stop()

        # ---------- 0.25 s logic ----------
        if current_time - prev >= .25:
            batch_max_conf = np.mean(batch_conf) if batch_conf else 0.0
            if np.isnan(batch_max_conf) or batch_max_conf < 0.5:
                batch_max_conf = 0.0

            if state == 1:
                if batch_max_conf >= 0.5:
                    enemy_pose = nav_bridge.estimate_enemy_pose(nav_bridge.get_front_range()) if nav_bridge else None
                    if nav_bridge and nav_bridge.sequence_active and nav_bridge.enemy_already_known(enemy_pose):
                        print("Detected enemy already recorded; ignoring and resuming Nav2.")
                        nav_bridge.mark_sequence_complete()
                        reset_state_machine()
                    else:
                        if nav_bridge and nav_bridge.sequence_active:
                            nav_bridge.pending_enemy_pose = enemy_pose
                        focus_pos = 750
                        object_detected = True
                        acquiring = True
                        print(f"Object detected with confidence {batch_max_conf:.2f}, switching to state 1→2 for focusing.")
                        current_index = 0
                        conf_max = 0.0
                        ideal_focus = focus_pos

            if state == 2:  # Autofocus sweep state
                if batch_max_conf >= conf_max:
                    conf_max = batch_max_conf
                    ideal_focus = focus_pos
                    print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                # move to next focus position safely
                current_index += 1
                if current_index >= len(focus_positions):
                    # finished full sweep
                    if conf_max < 0.5:
                        object_detected_once = False
                        focus_pos = 300
                        current_index = 0
                        state = 1
                        print("STATE -> 1")
                        print("Sweep done, low confidence. Returning to scan.")
                    else:
                        object_detected_once = False
                        focus_pos = ideal_focus
                        print(
                            f"Completed first full sweep after detection. "
                            f"Best focus: {ideal_focus} with conf: {conf_max}"
                        )
                        current_index = 0
                        state = 3
                        print("STATE -> 3")
                else:
                    focus_pos = focus_positions[current_index]

            if state == 4:
                print(state)
                focus_pos -= A
                if batch_max_conf <= past_conf - 0.01:
                    A = -A
                    print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")
                if batch_max_conf < (past_conf - (past_conf * error)):
                    state = 1
                    print("STATE -> 1")
                    focus_pos = 300
                    conf_max = 0
                    ideal_focus = 0
                    print(f"Cannot refine focus, returning to scan. Last good conf: {past_conf}")
                if batch_max_conf >= past_conf and batch_max_conf >= 0.5:
                    state = 5
                    print("STATE -> 5")
                    print(f"Object in focus with sufficient confidence. Holding focus at: {focus_pos} with conf: {batch_max_conf}")

            if state == 5:
                # dwell here some time before firing laser
                laser_toggle_count += 1
                if laser_toggle_count > 16:  # ~4 s at 0.25s per tick
                    laser_toggle_count = 0
                    state = 6
                    print("STATE -> 6")

            if state == 6:
                # LASER ON window
                if laser_toggle_count == 0:
                    GPIO.output(SERVO_PIN, GPIO.HIGH)  # turn laser ON once
                laser_toggle_count += 1
                print(6)
                if laser_toggle_count >= 40:  # ~10 s ON time
                    GPIO.output(SERVO_PIN, GPIO.LOW)   # turn laser OFF
                    laser_toggle_count = 0
                    state = 7
                    print("STATE -> 7")
                    motors.stop()

            if state == 7:
                # cool-down / re-enable motors, keep laser OFF
                laser_toggle_count += 1
                if laser_toggle_count >= 40:  # ~10 s cool-downo
                    if nav_bridge:
                        latest_enemy_pose = (
                            nav_bridge.pending_enemy_pose
                            or nav_bridge.estimate_enemy_pose(nav_bridge.get_front_range())
                        )
                        nav_bridge.record_enemy(latest_enemy_pose)
                    focus_pos = 300
                    laser_toggle_count = 0
                    GPIO.output(SERVO_PIN, GPIO.LOW)   # ensure laser stays OFF
                    state = 1
                    print("STATE -> 1")
                    conf_max = 0.0
                    acquiring = True
                    object_detected = False
                    angle = 30

            if state == 8:  # Autofocus sweep state
                if batch_max_conf >= conf_max:
                    conf_max = batch_max_conf
                    ideal_focus = focus_pos
                    print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                # move to next focus position safely
                current_index += 1
                if current_index >= len(focus_positions):
                    # finished full sweep
                    if conf_max < 0.5:
                        object_detected_once = False
                        focus_pos = 300
                        current_index = 0
                        state = 1
                        print("STATE -> 1")
                        print("Sweep done, low confidence. Returning to scan.")
                    else:
                        object_detected_once = False
                        focus_pos = ideal_focus
                        acquiring = True
                        print("Completed first full sweep after detection. " f"Best focus: {ideal_focus} with conf: {conf_max}")
                        current_index = 0

                        # --- store the servo angle at the moment we transition to state 9 ---
                        object_servo_angle = angle

                        state = 9
                        print("STATE -> 9")
                else:
                    focus_pos = focus_positions[current_index]

            if state == 10:
                focus_pos -= A
                if batch_max_conf <= past_conf - 0.01:
                    A = -A
                    print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")

            # update past_conf for next tick (used in state 4)
            focuser.set(Focuser.OPT_FOCUS, focus_pos)
            past_conf = batch_max_conf
            prev = current_time
            batch_conf.clear()

        if nav_bridge.sequence_active and state != 1:
            nav_bridge.sequence_progressed = True

        if (
            nav_bridge.sequence_active
            and nav_bridge.sequence_progressed
            and state == 1
            and previous_state != 1
        ):
            motors.stop()
            GPIO.output(SERVO_PIN, GPIO.LOW)
            nav_bridge.mark_sequence_complete()
            reset_state_machine()
            previous_state = state
            continue

        previous_state = state

finally:
    cap.release()
    cv2.destroyAllWindows()
    try:
        if executor is not None:
            executor.shutdown()
    except Exception:
        pass
    if nav_bridge is not None:
        try:
            nav_bridge.destroy_node()
        except Exception:
            pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    if spin_thread is not None and spin_thread.is_alive():
        spin_thread.join(timeout=1.0)


====================
File: run_cartographer.sh
====================
mkdir -p ~/Desktop/mapping_cartogropher

cat > ~/Desktop/mapping_cartogropher/run_cartographer.sh <<'EOF'
#!/usr/bin/env bash
set -e

# Source ROS 2 and your Cartographer workspace
source /opt/ros/humble/setup.bash
source ~/carto_ws/install/setup.bash

# Launch Cartographer with RPLIDAR and BNO055 IMU
ros2 launch ros2_mapping_support cartographer_lidar_imu.launch.py \
  lidar_port:=/dev/rplidar \
  i2c_bus:=7 \
  i2c_address:=0x28
EOF

chmod +x ~/Desktop/mapping_cartogropher/run_cartographer.sh


====================
File: tablet_nav2.py
====================
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TabletNavBridge(Node):
    def __init__(self):
        super().__init__('tablet_nav_bridge')

        # 1) Subscribe to the tablet goal topic (DO NOT change this name)
        self.sub = self.create_subscription(
            PoseStamped,
            '/tablet_goal',
            self.on_tablet_goal,
            10
        )

        # 2) Nav2 action client (standard nav2 action name)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('TabletNavBridge up, listening on /tablet_goal')

    def on_tablet_goal(self, msg: PoseStamped):
        # Log what we got from the tablet
        p = msg.pose.position
        o = msg.pose.orientation
        self.get_logger().info(
            f"Received tablet goal: "
            f"x={p.x:.2f}, y={p.y:.2f}, "
            f"q=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
        )

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 navigate_to_pose action server not available')
            return

        goal_msg = NavigateToPose.Goal()

        # 3) Copy pose straight through, but refresh the header stamp
        goal_msg.pose.header.frame_id = msg.header.frame_id or 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = msg.pose.position
        goal_msg.pose.pose.orientation = msg.pose.orientation  # <-- KEEP ORIENTATION

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected')
            return

        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Nav2 result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = TabletNavBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


====================
File: tools/wasd_teleop_cartographer.py
====================
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


====================
File: src/ros2_mapping_support/CMakeLists.txt
====================
cmake_minimum_required(VERSION 3.8)
project(ros2_mapping_support)

# Core ROS 2 build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python package (defined by setup.py / setup.cfg)
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes / scripts
install(
  PROGRAMS
    bno055_imu_node.py
    fake_odom_from_tf.py
    TabletPather.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install ALL configs (this is what fixes your error)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install resources (e.g. package marker)
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


====================
File: src/ros2_mapping_support/package.xml
====================
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2_mapping_support</name>
  <version>0.0.0</version>
  <description>RPLIDAR + BNO055 IMU support and Cartographer launch files</description>

  <maintainer email="gavintunnell20@gmail.com">team4</maintainer>
  <license>MIT</license>

  <!-- Python build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Build / run dependencies -->
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>

  <!-- We only need these at runtime, not to build this package -->
  <exec_depend>rplidar_ros</exec_depend>
  <exec_depend>cartographer_ros</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>robot_localization</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>


====================
File: src/ros2_mapping_support/setup.cfg
====================
[develop]
script_dir=$base/lib/ros2_mapping_support
[install]
install_scripts=$base/lib/ros2_mapping_support

[metadata]
name = ros2_mapping_support
version = 0.0.0
author = Team4
author_email = team4@example.com
description = Support nodes and configs for Cartographer + BNO055 + RPLidar
license = BSD-3-Clause

[options]
packages = find:
package_dir =
    = .
install_requires =
    setuptools
zip_safe = true

[options.packages.find]
where = .

[options.data_files]
share/ament_index/resource_index/packages =
    ros2_mapping_support
share/ros2_mapping_support =
    package.xml
share/ros2_mapping_support/launch =
    launch/full_system_cartographer_nav2.launch.py
    launch/full_system_slamtoolbox_nav2.launch.py
share/ros2_mapping_support/config =
    config/nav2_params_cartographer.yaml
    config/nav2_params_slamtoolbox.yaml
    config/nav2_slam_toolbox_params.yaml
    config/slam_toolbox_params.yaml
    config/cartographer_2d_no_odom.lua
    config/simple_nav.xml
    config/nav2_params.yaml



[options.entry_points]
console_scripts =
    bno055_imu_node = ros2_mapping_support.bno055_imu_node:main
    fake_odom_from_tf = ros2_mapping_support.fake_odom_from_tf:main


====================
File: src/ros2_mapping_support/setup.py
====================
from setuptools import setup

package_name = 'ros2_mapping_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Configs (install everything we ship so launches can find them)
        ('share/' + package_name + '/config', [
            'config/cartographer_2d_no_odom.lua',
            'config/nav2_params_cartographer_slam.yaml',
            'config/nav2_params.yaml',
            'config/nav2_slam_toolbox_params.yaml',
            'config/simple_nav.xml',
            'config/slam_toolbox_online_async.yaml',
            'config/slam_toolbox_params.yaml',
            'config/slam_params.yaml',
            'config/my_map.yaml',
            'config/my_map.pgm',
            'config/ekf_imu_only.yaml',
        ]),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/cartographer_lidar_imu.launch.py',
            'launch/full_system_cartographer_nav2.launch.py',
            'launch/full_system_slamtoolbox_nav2.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team4',
    maintainer_email='gavintunnell20@gmail.com',
    description='Support nodes and configs for Cartographer + Nav2 mapping',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_imu_node = ros2_mapping_support.bno055_imu_node:main',
            'motor_driver_pca_reg_dual = ros2_mapping_support.motor_driver_pca_reg_dual:main',
            'nav_alignment_state_machine = ros2_mapping_support.nav_alignment_state_machine:main',
        ],
    },
)


====================
File: src/ros2_mapping_support/bno055_imu_node.py
====================
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
        u16(offs_dict["mag_offset_x"]),
        u16(offs_dict["mag_offset_y"]),
        u16(offs_dict["mag_offset_z"]),
        u16(offs_dict["gyro_offset_x"]),
        u16(offs_dict["gyro_offset_y"]),
        u16(offs_dict["gyro_offset_z"]),
        u16(offs_dict["acc_radius"]),
        u16(offs_dict["mag_radius"]),
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


====================
File: src/ros2_mapping_support/ros2_mapping_support/__init__.py
====================



====================
File: src/ros2_mapping_support/ros2_mapping_support/bno055_imu_node.py
====================
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


====================
File: src/ros2_mapping_support/ros2_mapping_support/motor_driver_pca_reg_dual.py
====================
#!/usr/bin/env python3
# Dual-PCA9685 motor driver for /cmd_vel (no GPIO; register-level I2C)
#  - PCA @ 0x41: channels [EnA, In1, In2]  (default 0,1,2)
#  - PCA @ 0x60: channels [EnB, In3, In4]  (default 0,1,2)
# Forward logic (default):
#   Right side (EnA/In1/In2): forward => IN1=1, IN2=0
#   Left  side (EnB/In3/In4): forward => IN3=1, IN4=0
# STOP/coast = inputs LOW (both 0). If brake_on_zero=True, both HIGH.
#
# Notes:
#  - Use strings for I2C addresses ("0x41", "0x60") to avoid ROS param type issues
#  - Use `max_ang_cmd` so wz=max_ang_cmd gives full-power counter-rotation
#  - `min_duty_pct` ensures startup torque; typical 30–40% for DC gear motors
#
# Example:
#   source /opt/ros/humble/setup.bash
#   python3 motor_driver_pca_reg_dual.py --ros-args \
#     -p ena_addr:="'0x41'" -p enb_addr:="'0x60'" \
#     -p ena_channel:=0 -p in1_channel:=1 -p in2_channel:=2 \
#     -p enb_c  hannel:=0 -p in3_channel:=1 -p in4_channel:=2 \
#     -p pwm_freq_hz:=1000.0 -p min_duty_pct:=35.0 \
#     -p max_lin:=0.8 -p max_ang_cmd:=1.2 -p deadband:=0.03 \
#     -p invert_right:=true -p invert_left:=false

import math
import time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

# ---------- PCA9685 low-level ----------
_PCA_MODE1        = 0x00
_PCA_MODE2        = 0x01
_PCA_PRESCALE     = 0xFE
_PCA_LED0_ON_L    = 0x06
# MODE1 bits
_RESTART          = 0x80
_SLEEP            = 0x10
_AI               = 0x20
# MODE2 bits
_OUTDRV           = 0x04  # totem-pole

class PCA9685LowLevel:
    def __init__(self, busnum: int, address: int, freq_hz: float):
        self.bus    = smbus.SMBus(busnum)
        self.addr   = address
        self.freq_hz= float(freq_hz)
        self._init_chip()

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _init_chip(self):
        # reset
        self._write8(_PCA_MODE1, 0x00)  # all call off, AI off initially
        self._write8(_PCA_MODE2, _OUTDRV)
        time.sleep(0.005)
        # sleep to set prescale
        oldmode = self._read8(_PCA_MODE1)
        self._write8(_PCA_MODE1, (oldmode | _SLEEP) & 0xFF)
        prescale = int(round(25_000_000.0 / (4096.0 * self.freq_hz) - 1.0))
        prescale = max(3, min(255, prescale))
        self._write8(_PCA_PRESCALE, prescale)
        # wake, auto-increment
        self._write8(_PCA_MODE1, (oldmode & ~_SLEEP) | _AI)
        time.sleep(0.005)
        # restart
        self._write8(_PCA_MODE1, ((self._read8(_PCA_MODE1) | _RESTART) | _AI) & 0xFF)

    def set_pwm_raw(self, channel: int, on_count: int, off_count: int):
        base = _PCA_LED0_ON_L + 4 * channel
        self._write8(base + 0, on_count & 0xFF)
        self._write8(base + 1, (on_count >> 8) & 0x0F)
        self._write8(base + 2, off_count & 0xFF)
        self._write8(base + 3, (off_count >> 8) & 0x0F)

    def set_pwm_duty(self, channel: int, duty: float):
        # duty in [0.0, 1.0]
        duty = max(0.0, min(1.0, duty))
        if duty <= 0.0:
            # fully off
            self.set_pwm_raw(channel, 0, 0)
        elif duty >= 1.0:
            # fully on (use ON=0x1000 trick or set OFF=0)
            self.set_pwm_raw(channel, 0, 4095)
        else:
            off = int(round(duty * 4095))
            off = max(1, min(4094, off))
            self.set_pwm_raw(channel, 0, off)

    def set_pin_digital(self, channel: int, high: bool):
        self.set_pwm_duty(channel, 1.0 if high else 0.0)

# ---------- Helpers ----------
def _clip(x, lo, hi): return lo if x < lo else hi if x > hi else x

# ---------- ROS Node ----------
class MotorDriverPCADual(Node):
    def __init__(self):
        super().__init__('motor_driver_pca_dual')

        # Addresses as STRINGS so CLI -p works cleanly
        self.declare_parameter('ena_addr', '0x41')
        self.declare_parameter('enb_addr', '0x60')

        # Channel mapping (defaults: En*=0, inputs 1&2)
        self.declare_parameter('ena_channel', 0)
        self.declare_parameter('in1_channel', 1)
        self.declare_parameter('in2_channel', 2)

        self.declare_parameter('enb_channel', 0)
        self.declare_parameter('in3_channel', 1)
        self.declare_parameter('in4_channel', 2)

        # Control params
        self.declare_parameter('pwm_freq_hz', 1000.0)
        self.declare_parameter('min_duty_pct', 35.0)   # %
        self.declare_parameter('min_duty_turn_pct', 75.0) # % for turn-in-place
        self.declare_parameter('deadband', 0.03)       # normalized
        self.declare_parameter('max_lin', 0.8)         # m/s (for normalization)
        self.declare_parameter('max_ang_cmd', 1.2)     # rad/s that maps to full turn
        self.declare_parameter('brake_on_zero', False) # if True => both inputs HIGH on zero

        # Side inversion (flip forward)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_left',  False)

        # Which side EnA drives (True=left)
        self.declare_parameter('map_enA_to_left', True)

        # Command topic for Twist messages (teleop/Nav2)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # I2C bus number (Jetson 40-pin I2C is usually bus 1)
        self.declare_parameter('i2c_bus', 1)

        # Read params
        ena_addr_str = self.get_parameter('ena_addr').get_parameter_value().string_value
        enb_addr_str = self.get_parameter('enb_addr').get_parameter_value().string_value
        self.ena_addr = int(ena_addr_str, 16)
        self.enb_addr = int(enb_addr_str, 16)

        self.ena_ch = int(self.get_parameter('ena_channel').value)
        self.in1_ch = int(self.get_parameter('in1_channel').value)
        self.in2_ch = int(self.get_parameter('in2_channel').value)

        self.enb_ch = int(self.get_parameter('enb_channel').value)
        self.in3_ch = int(self.get_parameter('in3_channel').value)
        self.in4_ch = int(self.get_parameter('in4_channel').value)

        self.freq_hz   = float(self.get_parameter('pwm_freq_hz').value)
        self.min_duty  = float(self.get_parameter('min_duty_pct').value) / 100.0
        self.min_duty_turn = float(self.get_parameter('min_duty_turn_pct').value) / 100.0
        self.deadband  = float(self.get_parameter('deadband').value)
        self.max_lin   = float(self.get_parameter('max_lin').value)
        self.max_ang   = float(self.get_parameter('max_ang_cmd').value)
        self.brake0    = bool(self.get_parameter('brake_on_zero').value)

        self.inv_r     = bool(self.get_parameter('invert_right').value)
        self.inv_l     = bool(self.get_parameter('invert_left').value)
        self.mapA_left = bool(self.get_parameter('map_enA_to_left').value)

        cmd_topic_param = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.cmd_topic  = cmd_topic_param if cmd_topic_param else '/cmd_vel'

        busnum         = int(self.get_parameter('i2c_bus').value)

        # Init both PCA chips
        self.pcaA = PCA9685LowLevel(busnum, self.ena_addr, self.freq_hz)
        self.pcaB = PCA9685LowLevel(busnum, self.enb_addr, self.freq_hz)

        # Map sides to boards/channels
        if self.mapA_left:
            self.left  = dict(pca=self.pcaA, en=self.ena_ch, fwd=self.in3_ch, rev=self.in4_ch)  # use IN3/IN4 semantics for left
            self.right = dict(pca=self.pcaB, en=self.enb_ch, fwd=self.in1_ch, rev=self.in2_ch)  # use IN1/IN2 semantics for right
        else:
            self.left  = dict(pca=self.pcaB, en=self.enb_ch, fwd=self.in3_ch, rev=self.in4_ch)
            self.right = dict(pca=self.pcaA, en=self.ena_ch, fwd=self.in1_ch, rev=self.in2_ch)

        # Ensure stopped
        self._coast_side(self.left)
        self._coast_side(self.right)

        # Subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, qos)
        self.get_logger().info(
            f"PCA A=0x{self.ena_addr:02X} (EnA={self.ena_ch}, In1={self.in1_ch}, In2={self.in2_ch}) | "
            f"PCA B=0x{self.enb_addr:02X} (EnB={self.enb_ch}, In3={self.in3_ch}, In4={self.in4_ch}) | "
            f"freq={self.freq_hz}Hz min_duty={self.min_duty*100:.0f}% | listening on '{self.cmd_topic}'"
        )

    # ----- side helpers -----
    def _apply_side(self, side, forward: bool, duty: float, brake: bool):
        pca = side['pca']; en = side['en']; fwd = side['fwd']; rev = side['rev']
        duty = _clip(duty, 0.0, 1.0)

        if duty <= 0.0:
            # STOP
            pca.set_pwm_duty(en, 0.0)
            if self.brake0 or brake:
                pca.set_pin_digital(fwd, True)
                pca.set_pin_digital(rev, True)
            else:
                pca.set_pin_digital(fwd, False)
                pca.set_pin_digital(rev, False)
            return

        # Direction
        if forward:
            pca.set_pin_digital(fwd, True)
            pca.set_pin_digital(rev, False)
        else:
            pca.set_pin_digital(fwd, False)
            pca.set_pin_digital(rev, True)

        # Enable PWM
        pca.set_pwm_duty(en, duty)

    def _coast_side(self, side):
        side['pca'].set_pwm_duty(side['en'], 0.0)
        side['pca'].set_pin_digital(side['fwd'], False)
        side['pca'].set_pin_digital(side['rev'], False)

    # ----- mixer -----
    def on_cmd(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        # Normalize to [-1,1]
        mix_v = _clip(vx / max(1e-6, self.max_lin), -1.0, 1.0)
        mix_w = _clip(wz / max(1e-6, self.max_ang), -1.0, 1.0)

        r = mix_v + mix_w
        l = mix_v - mix_w

        # Keep within [-1,1] preserving ratio
        peak = max(1.0, abs(r), abs(l))
        r /= peak; l /= peak

        # Detect turn-in-place
        is_turn_in_place = abs(vx) < self.deadband and abs(wz) > 0

        def map_duty(x, is_turn):
            ax = abs(x)
            if ax < self.deadband:
                return 0.0, True

            min_d = self.min_duty_turn if is_turn else self.min_duty
            return max(min_d, ax), False

        duty_r, off_r = map_duty(r, is_turn_in_place)
        duty_l, off_l = map_duty(l, is_turn_in_place)

        # Apply inversion flags
        dir_r = (r >= 0.0)
        dir_l = (l >= 0.0)
        if self.inv_r: dir_r = not dir_r
        if self.inv_l: dir_l = not dir_l

        # Drive sides
        if off_r:
            self._apply_side(self.right, True, 0.0, brake=False)
        else:
            self._apply_side(self.right, dir_r, duty_r, brake=False)

        if off_l:
            self._apply_side(self.left, True, 0.0, brake=False)
        else:
            self._apply_side(self.left, dir_l, duty_l, brake=False)

    def destroy_node(self):
        try:
            self._coast_side(self.left)
            self._coast_side(self.right)
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = MotorDriverPCADual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


====================
File: src/ros2_mapping_support/ros2_mapping_support/nav_alignment_state_machine.py
====================

#!/usr/bin/env python3
import collections
import math
import os
from pathlib import Path
import threading
import time

import cv2
import Jetson.GPIO as GPIO
import numpy as np
import rclpy
import smbus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ultralytics import YOLO

from .focuser import Focuser


# ==== USER SETTINGS ====
BUS_NUM      = 1
ADDR_ENA     = 0x41
ADDR_ENB     = 0x60

ENA_CH       = 0
IN1_CH       = 1
IN2_CH       = 2
ENB_CH       = 0
IN3_CH       = 1
IN4_CH       = 2

PWM_FREQ_HZ  = 1000.0
PWM_FREQ_HZ_SERVO = 50.0
MIN_DUTY     = 0.00
MAX_DUTY     = 1.00

INVERT_RIGHT = True
INVERT_LEFT  = False
# ========================

_MODE1      = 0x00
_MODE2      = 0x01
_LED0_ON_L  = 0x06
_PRESCALE   = 0xFE

_RESTART = 1 << 7
_SLEEP   = 1 << 4
_AI      = 1 << 5
_OUTDRV  = 1 << 2


#Clip high/low values
def _clip(v, lo, hi):
    return hi if v > hi else lo if v < lo else v

#Get Duty cycle from angle
def angle_to_duty(angle, freq_hz=50.0):
    MIN_US = 500
    MAX_US = 2500
    angle = max(0, min(180, angle))
    us = MIN_US + (MAX_US - MIN_US) * angle / 180.0
    period_us = 1_000_000.0 / freq_hz
    duty = us / period_us    # fraction of total period
    return duty              # e.g. 0.075 for 7.5%

# ======================
# PCA9685 CLASS
# ======================
class PCA9685:
    def __init__(self, bus, addr, freq_hz=300.0):
        self.bus = bus
        self.addr = addr
        self._w8(_MODE1, _SLEEP | _AI)
        self._w8(_MODE2, _OUTDRV)
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old & ~_SLEEP) | _RESTART | _AI)
        time.sleep(0.005)

    def set_pwm_freq(self, f_hz):
        prescale = int(round(25_000_000.0 / (4096.0 * float(f_hz)) - 1.0))
        prescale = _clip(prescale, 3, 255)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old | _SLEEP) & 0x7F)
        self._w8(_PRESCALE, prescale)
        self._w8(_MODE1, old | _AI)
        time.sleep(0.005)
        self._w8(_MODE1, old | _AI | _RESTART)

    def set_off(self, ch):
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0,0,0x10])

    def set_on(self, ch):
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0x10,0,0])

    def set_pwm(self, ch, duty01):
        duty01 = _clip(duty01, 0.0, 1.0)
        if duty01 <= 0.0: self.set_off(ch); return
        if duty01 >= 1.0: self.set_on(ch);  return
        off = int(round(duty01 * 4095.0))
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0,0, off & 0xFF, (off>>8)&0x0F])

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _r8(self, reg):
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

# ======================
# MOTOR DRIVER CLASS
#Initializes the right and lefft motor drivers
# ======================
class MotorDriver:
    def __init__(self):
        self.bus = smbus.SMBus(BUS_NUM)
        self.pcaA = PCA9685(self.bus, ADDR_ENA, PWM_FREQ_HZ)
        self.pcaB = PCA9685(self.bus, ADDR_ENB, PWM_FREQ_HZ)

        self.invR = INVERT_RIGHT
        self.invL = INVERT_LEFT

        self.stop()

    # ===== Direction Helpers =====
    def _right_forward(self):
        if not self.invR:
            self.pcaA.set_on(IN1_CH); self.pcaA.set_off(IN2_CH)
        else:
            self.pcaA.set_off(IN1_CH); self.pcaA.set_on(IN2_CH)

    def _right_reverse(self):
        if not self.invR:
            self.pcaA.set_off(IN1_CH); self.pcaA.set_on(IN2_CH)
        else:
            self.pcaA.set_on(IN1_CH); self.pcaA.set_off(IN2_CH)

    def _left_forward(self):
        if not self.invL:
            self.pcaB.set_on(IN3_CH); self.pcaB.set_off(IN4_CH)
        else:
            self.pcaB.set_off(IN3_CH); self.pcaB.set_on(IN4_CH)

    def _left_reverse(self):
        if not self.invL:
            self.pcaB.set_off(IN3_CH); self.pcaB.set_on(IN4_CH)
        else:
            self.pcaB.set_on(IN3_CH); self.pcaB.set_off(IN4_CH)

    # ===== Motor Enable =====
    def _enable_both(self, duty):
        duty = _clip(duty, 0.0, 1.0)
        if duty == 0:
            self.pcaA.set_off(ENA_CH)
            self.pcaB.set_off(ENB_CH)
        else:
            self.pcaA.set_pwm(ENA_CH, duty)
            self.pcaB.set_pwm(ENB_CH, duty)

    # ===== PUBLIC =====
    def stop(self):
        self.pcaA.set_off(IN1_CH); self.pcaA.set_off(IN2_CH)
        self.pcaB.set_off(IN3_CH); self.pcaB.set_off(IN4_CH)
        self._enable_both(0)

    def brake(self):
        self.pcaA.set_on(IN1_CH); self.pcaA.set_on(IN2_CH)
        self.pcaB.set_on(IN3_CH); self.pcaB.set_on(IN4_CH)
        self._enable_both(0)

    def tank_drive(self, left, right):
        """
        left, right ∈ [-1, 1]
        sign determines direction
        0 => that side is OFF (no drive)
        """
        # Clip commands
        left  = _clip(left,  -1, 1)
        right = _clip(right, -1, 1)

        # ----- LEFT SIDE -----
        if abs(left) < 1e-3:
            # turn LEFT motor fully off (both inputs low)
            left_duty = 0.0
            self.pcaB.set_off(IN3_CH)
            self.pcaB.set_off(IN4_CH)
        else:
            left_sign  = 1 if left > 0 else -1
            left_duty  = _clip(abs(left), MIN_DUTY, MAX_DUTY)
            if left_sign > 0:
                self._left_forward()
            else:
                self._left_reverse()

        # ----- RIGHT SIDE -----
        if abs(right) < 1e-3:
            # turn RIGHT motor fully off (both inputs low)
            right_duty = 0.0
            self.pcaA.set_off(IN1_CH)
            self.pcaA.set_off(IN2_CH)
        else:
            right_sign = 1 if right > 0 else -1
            right_duty = _clip(abs(right), MIN_DUTY, MAX_DUTY)
            if right_sign > 0:
                self._right_forward()
            else:
                self._right_reverse()

        # Enable both sides with the higher duty
        self._enable_both(max(left_duty, right_duty))

    def arcade_drive(self, v, w):
        left  = _clip(v + w, -1, 1)
        right = _clip(v - w, -1, 1)
        self.tank_drive(left, right)

    # ===== Simple Motion Helpers (for WASD) =====
    def forward(self):
        self.tank_drive(-1, -1)

    def reverse(self):
        self.tank_drive(1, 1)

    def turn_left(self):
        self.tank_drive(-1, 0)

    def turn_right(self):
        self.tank_drive(0, -1)

    def rotate_right(self):
        self.tank_drive(0.8, -0.8)

    def rotate_left(self):
        self.tank_drive(-0.8, 0.8)

    def shutdown(self):
        try:
            self.stop()
            self.bus.close()
        except:
            pass


# ROS 2 / Nav2 integration to intercept goals, pause Nav2, run the manual
# alignment/laser sequence, and then resume the saved goal.
class Nav2Interrupter(Node):
    def __init__(self):
        super().__init__("nav2_manual_alignment_bridge")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cancel_client = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.goal_subs = [
            self.create_subscription(PoseStamped, "goal_pose", self._goal_cb, 10),
            self.create_subscription(PoseStamped, "move_base_simple/goal", self._goal_cb, 10),
        ]
        self.pending_goal = None
        self.sequence_active = False
        self.sequence_progressed = False
        self.reset_requested = False

    def _goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            "Captured navigation goal; canceling Nav2 and starting manual alignment sequence."
        )
        self.pending_goal = msg
        self.sequence_active = True
        self.sequence_progressed = False
        self.reset_requested = True
        self._cancel_nav2_goal()

    def _cancel_nav2_goal(self):
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "Nav2 cancel service unavailable; Nav2 may keep moving until the action server stops."
            )
            return
        req = CancelGoal.Request()
        self.cancel_client.call_async(req)

    def mark_sequence_complete(self):
        self.sequence_active = False
        self.sequence_progressed = False
        self.reset_requested = False
        self._resume_nav()

    def _resume_nav(self):
        if self.pending_goal is None:
            self.get_logger().warn("No stored goal to resume.")
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                "Nav2 action server not ready; keeping goal queued until the next completion event."
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.pending_goal
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to send resume goal: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the resume goal.")
            return

        self.get_logger().info("Nav2 goal accepted; navigation resumed toward the original destination.")
        self.pending_goal = None


def main():
    #Initializes the servo motors
    i2c_bus = smbus.SMBus(BUS_NUM)   # BUS_NUM is 1 in your settings

    # Now create the servo driver using that bus
    servo = PCA9685(bus=i2c_bus, addr=0x42, freq_hz=PWM_FREQ_HZ_SERVO)

    # ---------- Jetson Nano Camera GStreamer Pipeline ----------
    GST = (
        "nvarguscamerasrc sensor-id=0 !"
        "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 !"
        "nvvidconv !"
        "video/x-raw,format=BGRx !"
        "videoconvert ! video/x-raw,format=BGR !"
        "appsink drop=true max-buffers=4 sync=false"
    )

    SERVO_PIN = 22

    GPIO.setmode(GPIO.BOARD)            # Sets up Laser GPIO and sets output to LOW
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    GPIO.output(SERVO_PIN, GPIO.LOW)

    # -------- Model config --------
    env_engine = os.getenv("YOLO_ENGINE_PATH")
    engine_candidates = []
    if env_engine:
        engine_candidates.append(Path(env_engine).expanduser())
    workspace_engine = Path(__file__).resolve().parents[3] / "green_specific.engine"
    engine_candidates.append(workspace_engine)
    engine_candidates.append(Path.home() / "Camera" / "green_specific.engine")

    DEVICE      = 0
    IMGSZ       = 704
    CONF        = 0.25
    HALF        = True

    # -------- Load engine (explicit task to silence warning) --------
    engine_path = next((p for p in engine_candidates if p.exists()), None)
    if engine_path is None:
        raise SystemExit(
            "YOLO engine not found. Looked for (in order): "
            + ", ".join(str(p) for p in engine_candidates)
        )
    print(f"Loading YOLO engine from {engine_path}")
    model = YOLO(str(engine_path), task="detect")

    # -------- Camera --------
    cap = cv2.VideoCapture(GST, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Failed to open camera.")

    WIN = "IMX519 YOLOv8 TensorRT"
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    times = collections.deque(maxlen=60)
    prev  = time.time()

    # ---------- State variables ----------
    focuser = Focuser(bus=10)

    # State 1 variables
    state = 1
    print("STATE INIT -> 1")
    focus_pos = 300
    conf_max = 0.0
    servo_offset = 30        #Adjust servo offset if needed
    acquiring = True
    object_detected = False  #Object detection fields
    angle = 0
    batch_conf = []

    # State 2 variables
    focus_positions = list(range(650, 0, -50))  # [750, 700, ..., 50]
    current_index = 0
    ideal_focus = 300
    object_detected_once = False

    # State 3 Variables
    acquire_tol = 150          # px: stay in sweep until |error| <= this
    track_tol   = 10           # px: no movement if within this band (deadband)
    direction = 1
    SWEEP_STEP_DEG = 2.0       # degrees per sweep step (increase to pan faster)

    # State 4 Variables
    past_conf = 0.0
    error = .30
    A = 5  # focus step size (change if you want faster/slower focus sweeps)

    # State 5 Variables
    CENTER_MIN = 0
    CENTER_MAX = 180
    px_cushion   = 30          # cushion in pixels around image center
    slow_step    = 0.5         # degrees per nudge to keep motion slow

    # State 6 Variables
    laser_toggle_count = 0

    #State 9 Variables
    object_servo_angle = 0   # record servo angle when entering state 9


    # Initialize motors for the manual alignment/approach sequence
    motors = MotorDriver()

    #Sets initial servo position
    duty = angle_to_duty(angle)
    servo.set_pwm(0, duty)

    time.sleep(1)


    nav_bridge = None
    executor = None
    spin_thread = None


    try:
        rclpy.init(args=None)
        nav_bridge = Nav2Interrupter()
        executor = MultiThreadedExecutor()
        executor.add_node(nav_bridge)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        def reset_state_machine():
            nonlocal state, focus_pos, conf_max, acquiring, object_detected, angle
            nonlocal current_index, ideal_focus, object_detected_once, direction
            nonlocal past_conf, laser_toggle_count, object_servo_angle, prev

            state = 1
            focus_pos = 300
            conf_max = 0.0
            acquiring = True
            object_detected = False
            angle = 0
            batch_conf.clear()
            current_index = 0
            ideal_focus = 300
            object_detected_once = False
            direction = 1
            past_conf = 0.0
            laser_toggle_count = 0
            object_servo_angle = 0
            prev = time.time()
            motors.stop()
            duty_reset = angle_to_duty(angle + servo_offset)
            servo.set_pwm(0, duty_reset)
            print("STATE -> 1 (reset for new Nav2 goal)")

        previous_state = state

        while True:
            if nav_bridge.reset_requested:
                reset_state_machine()
                nav_bridge.reset_requested = False

            ok, frame_bgr = cap.read()
            if not ok:
                break

            if not nav_bridge.sequence_active:
                cv2.imshow(WIN, frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                previous_state = state
                continue

            results = model.predict(
                source=frame_bgr,
                device=DEVICE,
                imgsz=IMGSZ,
                conf=CONF,
                half=HALF,
                verbose=False
            )

            frame_width = frame_bgr.shape[1]
            center_x = frame_width / 2
            tolerance = 20

            r = results[0]

            current_time = time.time()

            if (0.1 <= (current_time - prev) < 0.25):
                if len(r.boxes.conf) > 0:
                    batch_conf.append(float(r.boxes.conf.max()))
                else:
                    batch_conf.append(0.0)

            # top-conf x_center in pixels
            x_center = float(r.boxes.xywh[r.boxes.conf.argmax(), 0]) if len(r.boxes) > 0 else None

            # normalized center for TOY SOLDIER (prefer), else top-conf box
            x_center_n = None

            if len(r.boxes) > 0:
                try:
                    names = r.names if hasattr(r, "names") else model.names
                    cls_ids = r.boxes.cls.cpu().numpy().astype(int)
                    xywhn   = r.boxes.xywhn.cpu().numpy()
                    confs   = r.boxes.conf.cpu().numpy()

                    # try to pick "toy soldier" if present
                    target_idx = None
                    if names is not None:
                        for i, cid in enumerate(cls_ids):
                            if str(names.get(int(cid), "")).lower() == "toy soldier":
                                if target_idx is None or confs[i] > confs[target_idx]:
                                    target_idx = i
                    # fallback to highest confidence
                    if target_idx is None:
                        target_idx = int(np.argmax(confs))

                    x_center_n = float(xywhn[target_idx, 0])  # normalized 0..1
                except Exception:
                    if x_center is not None and frame_width > 0:
                        x_center_n = float(x_center / frame_width)

            annotated = results[0].plot()
            cv2.imshow(WIN, annotated)

            # IMPORTANT: let OpenCV process GUI events
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break

            # ---------- Per-frame state logic ----------
            if state == 1:
                acquiring = True
                if angle > 180:
                    angle = 180
                    direction = -1
                elif angle < 0:
                    angle = 0
                    direction = 1

                # sweep
                angle += SWEEP_STEP_DEG * direction

                duty = angle_to_duty(angle + servo_offset)
                servo.set_pwm(0, duty)

                if object_detected:
                    state = 8
                    print("STATE -> 8")
                    motors.stop()

            if state == 3:
                if x_center is not None:
                    error_pixels = center_x - x_center  # + => target left of center

                    if acquiring and abs(error_pixels) > acquire_tol:
                        # keep sweeping in the current direction
                        angle += SWEEP_STEP_DEG * direction
                    else:
                        acquiring = False
                        if abs(error_pixels) > track_tol:
                            state = 4  # switch to fine-tracking state
                            print("STATE -> 4")
                else:
                    acquiring = True
                    angle += SWEEP_STEP_DEG * direction

                duty = angle_to_duty(angle + servo_offset)
                servo.set_pwm(0, duty)

            if state == 5:
                if x_center_n is not None and frame_width > 0:
                    margin_n = float(px_cushion) / float(frame_width)
                    # nudge slowly toward the center
                    if x_center_n < (0.5 - margin_n):
                        angle += slow_step     # target is left in image → turn left
                    elif x_center_n > (0.5 + margin_n):
                        angle -= slow_step     # target is right in image → turn right
                    # clamp to mechanical limits
                    if angle > CENTER_MAX:
                        angle = CENTER_MAX
                    elif angle < CENTER_MIN:
                        angle = CENTER_MIN
                # if no detection, just hold current angle
                duty = angle_to_duty(angle + servo_offset)
                servo.set_pwm(0, duty)

            if state == 9:
                print(state)
                # lock servo to 110 deg and sync logical angle
                angle = 110 - servo_offset
                duty = angle_to_duty(angle + servo_offset)
                servo.set_pwm(0, duty)

                # --- motor correction runs together with servo motion ---
                if x_center is not None:
                    error_pixels = center_x - x_center  # + => target left of center

                    if acquiring and abs(error_pixels) > acquire_tol:
                        # sweeping: rotate in the sweep direction
                        if error_pixels > acquire_tol:
                            # target is left in the image → rotate to reduce error
                            motors.rotate_left()
                        elif error_pixels < -acquire_tol:
                            #target is right in the image
                            motors.rotate_right()

                    else:
                        acquiring = False

                        # Fine tracking threshold → go to state 4
                        if abs(error_pixels) > track_tol:
                            # Rotate toward error BEFORE switching
                            if error_pixels > 0:
                                print("3")
                                motors.rotate_right()
                            else:
                                print("4")
                                motors.rotate_left()

                                state = 10
                                print("STATE -> 10")
                        else:
                            motors.stop()  # aligned with target
                else:
                    # No target: sweep
                    acquiring = True
                    if object_servo_angle > 110:
                        motors.rotate_left()
                    elif object_servo_angle < 110:
                        motors.rotate_right()

            if state == 10:
                motors.forward()
                boxes = r.boxes  # all detected boxes

                if boxes is not None and len(boxes) > 0:
                    # take the biggest object
                    biggest = max(boxes, key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) *
                                          (b.xyxy[0][3] - b.xyxy[0][1]))

                    x1, y1, x2, y2 = biggest.xyxy[0]  # tensor → floats

                    box_area = (x2 - x1) * (y2 - y1)
                    frame_area = 704 * 704
                    fill_ratio = box_area / frame_area   # value between 0 and 1

                    # WHERE THE ROBOT DRIFTS → send back to STATE 8
                    error_pixels = center_x - ((x1 + x2) / 2)

                    # >>> ONLY THIS LINE CHANGED <<<
                    if abs(error_pixels) > track_tol * 20:   # was * 2
                        motors.stop()
                        state = 8  # ← return to reacquire alignment
                        print("STATE -> 8")
                        continue

                    # Close enough
                    if fill_ratio >= 0.30:
                        motors.stop()
                        state = 2
                        print("STATE -> 2")

            # ---------- 0.25 s logic ----------
            if current_time - prev >= .25:
                batch_max_conf = np.mean(batch_conf) if batch_conf else 0.0
                if np.isnan(batch_max_conf) or batch_max_conf < 0.5:
                    batch_max_conf = 0.0

                if state == 1:
                    if batch_max_conf >= 0.5:
                        focus_pos = 750
                        object_detected = True
                        acquiring = True
                        print(f"Object detected with confidence {batch_max_conf:.2f}, switching to state 1→2 for focusing.")
                        current_index = 0
                        conf_max = 0.0
                        ideal_focus = focus_pos

                if state == 2:  # Autofocus sweep state
                    if batch_max_conf >= conf_max:
                        conf_max = batch_max_conf
                        ideal_focus = focus_pos
                        print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                    # move to next focus position safely
                    current_index += 1
                    if current_index >= len(focus_positions):
                        # finished full sweep
                        if conf_max < 0.5:
                            object_detected_once = False
                            focus_pos = 300
                            current_index = 0
                            state = 1
                            print("STATE -> 1")
                            print("Sweep done, low confidence. Returning to scan.")
                        else:
                            object_detected_once = False
                            focus_pos = ideal_focus
                            print(
                                f"Completed first full sweep after detection. "
                                f"Best focus: {ideal_focus} with conf: {conf_max}"
                            )
                            current_index = 0
                            state = 3
                            print("STATE -> 3")
                    else:
                        focus_pos = focus_positions[current_index]

                if state == 4:
                    print(state)
                    focus_pos -= A
                    if batch_max_conf <= past_conf - 0.01:
                        A = -A
                        print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")
                    if batch_max_conf < (past_conf - (past_conf * error)):
                        state = 1
                        print("STATE -> 1")
                        focus_pos = 300
                        conf_max = 0
                        ideal_focus = 0
                        print(f"Cannot refine focus, returning to scan. Last good conf: {past_conf}")
                    if batch_max_conf >= past_conf and batch_max_conf >= 0.5:
                        state = 5
                        print("STATE -> 5")
                        print(f"Object in focus with sufficient confidence. Holding focus at: {focus_pos} with conf: {batch_max_conf}")

                if state == 5:
                    # dwell here some time before firing laser
                    laser_toggle_count += 1
                    if laser_toggle_count > 16:  # ~4 s at 0.25s per tick
                        laser_toggle_count = 0
                        state = 6
                        print("STATE -> 6")

                if state == 6:
                    # LASER ON window
                    if laser_toggle_count == 0:
                        GPIO.output(SERVO_PIN, GPIO.HIGH)  # turn laser ON once
                    laser_toggle_count += 1
                    print(6)
                    if laser_toggle_count >= 40:  # ~10 s ON time
                        GPIO.output(SERVO_PIN, GPIO.LOW)   # turn laser OFF
                        laser_toggle_count = 0
                        state = 7
                        print("STATE -> 7")
                        motors.stop()

                if state == 7:
                    # cool-down / re-enable motors, keep laser OFF
                    laser_toggle_count += 1
                    if laser_toggle_count >= 40:  # ~10 s cool-downo
                        focus_pos = 300
                        laser_toggle_count = 0
                        GPIO.output(SERVO_PIN, GPIO.LOW)   # ensure laser stays OFF
                        state = 1
                        print("STATE -> 1")
                        conf_max = 0.0
                        acquiring = True
                        object_detected = False
                        angle = 30

                if state == 8:  # Autofocus sweep state
                    if batch_max_conf >= conf_max:
                        conf_max = batch_max_conf
                        ideal_focus = focus_pos
                        print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                    # move to next focus position safely
                    current_index += 1
                    if current_index >= len(focus_positions):
                        # finished full sweep
                        if conf_max < 0.5:
                            object_detected_once = False
                            focus_pos = 300
                            current_index = 0
                            state = 1
                            print("STATE -> 1")
                            print("Sweep done, low confidence. Returning to scan.")
                        else:
                            object_detected_once = False
                            focus_pos = ideal_focus
                            acquiring = True
                            print("Completed first full sweep after detection. " f"Best focus: {ideal_focus} with conf: {conf_max}")
                            current_index = 0

                            # --- store the servo angle at the moment we transition to state 9 ---
                            object_servo_angle = angle

                            state = 9
                            print("STATE -> 9")
                    else:
                        focus_pos = focus_positions[current_index]

                if state == 10:
                    focus_pos -= A
                    if batch_max_conf <= past_conf - 0.01:
                        A = -A
                        print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")

                # update past_conf for next tick (used in state 4)
                focuser.set(Focuser.OPT_FOCUS, focus_pos)
                past_conf = batch_max_conf
                prev = current_time
                batch_conf.clear()

            if nav_bridge.sequence_active and state != 1:
                nav_bridge.sequence_progressed = True

            if (
                nav_bridge.sequence_active
                and nav_bridge.sequence_progressed
                and state == 1
                and previous_state != 1
            ):
                motors.stop()
                GPIO.output(SERVO_PIN, GPIO.LOW)
                nav_bridge.mark_sequence_complete()
                reset_state_machine()
                previous_state = state
                continue

            previous_state = state

    finally:
        motors.shutdown()
        try:
            i2c_bus.close()
        except Exception:
            pass
        GPIO.output(SERVO_PIN, GPIO.LOW)
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()
        if nav_bridge is not None:
            try:
                nav_bridge.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        try:
            if executor is not None:
                executor.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()


====================
File: src/ros2_mapping_support/launch/cartographer_lidar_imu.launch.py
====================
#!/usr/bin/env python3
"""
Cartographer + sensors bringup:

- RPLidar + BNO055
- Static TFs to base_link
- Cartographer 2D + occupancy grid
- Optional RViz2 config (if present in this package)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package share (for configs)
    pkg_share = get_package_share_directory("ros2_mapping_support")

    # Optional RViz config; if missing, RViz starts empty
    rviz_config = os.path.join(pkg_share, "rviz_cartographer_nav2.rviz")
    rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []

    # Launch arguments
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RPLidar (e.g. /dev/ttyUSB0 or /dev/rplidar)",
    )
    declare_i2c_bus = DeclareLaunchArgument(
        "i2c_bus",
        default_value="7",
        description="I2C bus index for BNO055",
    )
    declare_i2c_address = DeclareLaunchArgument(
        "i2c_address",
        default_value="0x28",
        description="I2C address for BNO055 (e.g. 0x28)",
    )

    # Environment
    env_fastdds = SetEnvironmentVariable(
        name="FASTDDS_TRANSPORT_SHARED_MEM", value="off"
    )

    # RPLidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_port"),
            "serial_baudrate": 115200,
            "frame_id": "laser_frame",
            "inverted": False,
            "angle_compensate": True,
        }],
    )

    # IMU node (BNO055)
    imu_node = Node(
        package="ros2_mapping_support",
        executable="bno055_imu_node",
        name="bno055_imu_node",
        output="screen",
        parameters=[{
            "i2c_bus": LaunchConfiguration("i2c_bus"),
            "i2c_address": LaunchConfiguration("i2c_address"),
        }],
    )

    # Static transforms
    static_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "0.120", "0.0", "0.100",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "laser_frame",
        ],
        output="screen",
    )

    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "imu_link",
        ],
        output="screen",
    )

    # Cartographer 2D + occupancy grid
    carto_config_dir = os.path.join(pkg_share, "config")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory", carto_config_dir,
            "-configuration_basename", "cartographer_2d_no_odom.lua",
        ],
        parameters=[{"use_sim_time": False}],
        remappings=[
            ("scan", "scan"),
            ("imu", "imu/data"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "resolution": 0.05,
            "publish_period_sec": 0.3,
        }],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
    )

    return LaunchDescription([
        env_fastdds,
        declare_lidar_port,
        declare_i2c_bus,
        declare_i2c_address,
        lidar_node,
        imu_node,
        static_laser_tf,
        static_imu_tf,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])


====================
File: src/ros2_mapping_support/launch/fake_odom_from_tf.py
====================
#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster
from geometry_msgs.msg import Quaternion, Point
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Extract yaw from a geometry_msgs/Quaternion.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FakeOdomFromTF(Node):
    """
    Periodically looks up TF (map -> base_link) and publishes nav_msgs/Odometry
    on /odom so external tools (tablet) can consume a simple odom topic.

    NOTE: we are using 'map' as the fixed frame. If you later configure
    Cartographer to provide a real 'odom' frame, just change the lookup
    target_frame from 'map' to 'odom' below and set header.frame_id = 'odom'.
    """

    def __init__(self):
        super().__init__("fake_odom_from_tf")

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry publisher (mirrors TF as /odom)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Timer at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(
            "fake_odom_from_tf: publishing /odom from TF (map -> base_link)"
        )

    def timer_cb(self):
        try:
            # *** IMPORTANT ***
            # Using map as the fixed frame here (Option B).
            # If you later have a real 'odom' frame, change "map" -> "odom".
            t = self.tf_buffer.lookup_transform(
                "map",        # target_frame (fixed)
                "base_link",  # source_frame (robot base)
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get TF map -> base_link: {ex}", throttle_duration_sec=5.0
            )
            return

        # Fill Odometry message
        msg = Odometry()
        msg.header.stamp = t.header.stamp
        # We keep frame_id = "map" to match the TF tree
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        # Pose from TF
        msg.pose.pose.position = Point()  # Create an instance of Point
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z

        msg.pose.pose.orientation = t.transform.rotation  # Quaternion from transform

        # Covariances: simple reasonable defaults
        msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.1,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.1,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1,
        ]

        # We don't have velocity info from TF, so leave twist = 0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(msg)

        # Also broadcast a TF using fake_odom as the parent
        tf_msg = TransformStamped()
        tf_msg.header.stamp = t.header.stamp
        tf_msg.header.frame_id = "fake_odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation = t.transform.translation
        tf_msg.transform.rotation = t.transform.rotation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomFromTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


====================
File: src/ros2_mapping_support/launch/full_system_cartographer_nav2.launch.py
====================
#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Launch arguments ----
    lidar_port  = LaunchConfiguration("lidar_port")
    i2c_bus     = LaunchConfiguration("i2c_bus")
    i2c_address = LaunchConfiguration("i2c_address")
    rviz_config = LaunchConfiguration("rviz_config")
    nav2_params = LaunchConfiguration("nav2_params")

    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="RPLidar serial port (e.g. /dev/ttyUSB0)",
    )
    declare_i2c_bus = DeclareLaunchArgument(
        "i2c_bus",
        default_value="7",
        description="I2C bus index for BNO055",
    )
    declare_i2c_address = DeclareLaunchArgument(
        "i2c_address",
        default_value="0x28",
        description="I2C address for BNO055 (e.g. 0x28)",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("ros2_mapping_support"),
            "config",
            "default.rviz",
        ),
        description="Full path to an RViz2 config file",
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=os.path.join(
            get_package_share_directory("ros2_mapping_support"),
            "config",
            "nav2_params_cartographer_slam.yaml",
        ),
        description="Nav2 params file for Cartographer SLAM/AMCL run",
    )

    # Turn off shared memory for FastDDS (needed for your motor driver)
    disable_shm = SetEnvironmentVariable(
        name="FASTDDS_TRANSPORT_SHARED_MEM",
        value="off",
    )

    pkg_share = get_package_share_directory("ros2_mapping_support")

    # ---- RPLidar node ----
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": lidar_port,
            "serial_baudrate": 115200,
            "frame_id": "laser_frame",
            "inverted": False,
            "angle_compensate": True,
        }],
    )

    # ---- IMU node (BNO055, IMUPLUS mode, no mag) ----
    imu_node = Node(
        package="ros2_mapping_support",
        executable="bno055_imu_node",
        name="bno055_imu_node",
        output="screen",
        parameters=[{
            "i2c_bus": i2c_bus,
            "i2c_address": i2c_address,
        }],
    )

    # ---- Static transforms ----
    static_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "0.120", "0.0", "0.100",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "laser_frame",
        ],
        output="screen",
    )

    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "imu_link",
        ],
        output="screen",
    )

    # ---- Cartographer 2D + occupancy grid ----
    carto_config_dir = os.path.join(pkg_share, "config")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory", carto_config_dir,
            "-configuration_basename", "cartographer_2d_no_odom.lua",
        ],
        parameters=[{"use_sim_time": False}],
        remappings=[
            ("scan", "scan"),
            ("imu", "imu/data"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "resolution": 0.05,
            "publish_period_sec": 0.3,
        }],
    )

    # ---- Nav2 bringup (use AMCL + map server with Cartographer map) ----
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("nav2_bringup"),
            "launch",
            "navigation_launch.py",
        )
    ),
    launch_arguments={
        "use_sim_time": "false",
        "autostart": "true",
        "params_file": nav2_params,
        "slam": "False",
    }.items(),
)

    # ---- Motor driver ----
    motor_driver_node = Node(
        package="ros2_mapping_support",
        executable="motor_driver_pca_reg_dual",
        name="motor_driver",
        output="screen",
        parameters=[{
            "ena_addr": "0x41",
            "enb_addr": "0x60",
            "ena_channel": 0,
            "in1_channel": 1,
            "in2_channel": 2,
            "enb_channel": 0,
            "in3_channel": 1,
            "in4_channel": 2,
            "pwm_freq_hz": 1000.0,
            "max_lin": 1.1,
            "max_ang_cmd": 1.5,
            "deadband": 0.03,
            "min_duty_pct": 55.0,
            "brake_on_zero": False,
            "invert_right": True,
            "invert_left": False,
            "map_enA_to_left": True,
        }],
    )

    # ---- RViz2 ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        disable_shm,
        declare_lidar_port,
        declare_i2c_bus,
        declare_i2c_address,
        declare_rviz_config,
        declare_nav2_params,
        lidar_node,
        imu_node,
        static_laser_tf,
        static_imu_tf,
        cartographer_node,
        occupancy_grid_node,
        nav2_bringup_launch,
        motor_driver_node,
        rviz_node,
    ])


====================
File: src/ros2_mapping_support/config/cartographer_2d_no_odom.lua
====================
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Cartographer publishes odom -> base_link; AMCL will own map -> odom.
  map_frame = "map",
  tracking_frame = "base_link",   -- imu_link is colocated via TF
  published_frame = "base_link",
  odom_frame = "odom",

  -- Publish an odom frame so downstream consumers always see map->odom->base_link.
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.005,
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D trajectory builder tuning
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Slightly stronger motion filter
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Pose graph tuning
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options


====================
File: src/ros2_mapping_support/config/ekf_imu_only.yaml
====================
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 1.0
    two_d_mode: true

    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # IMU input only (orientation + angular velocity)
    imu0: /imu/data
    imu0_config: [
      false, false, false,   # x y z
      true,  true,  true,    # roll pitch yaw
      false, false, false,   # vx vy vz
      true,  true,  true,    # vroll vpitch vyaw
      false, false, false    # ax ay az
    ]
    imu0_nodelay: false
    imu0_differential: true
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true

    # Process/measurement noise (light defaults; tune as needed)
    process_noise_covariance: [
      1.0e-2, 0.0,    0.0,    0.0,    0.0,    0.0,
      0.0,    1.0e-2, 0.0,    0.0,    0.0,    0.0,
      0.0,    0.0,    1.0e-2, 0.0,    0.0,    0.0,
      0.0,    0.0,    0.0,    1.0e-3, 0.0,    0.0,
      0.0,    0.0,    0.0,    0.0,    1.0e-3, 0.0,
      0.0,    0.0,    0.0,    0.0,    0.0,    1.0e-2
    ]

    # Let EKF use the IMU message covariances for measurements (orientation/gyro)

    imu0_orientation_rejection_threshold: 0.8
    imu0_angular_velocity_rejection_threshold: 0.8


====================
File: src/ros2_mapping_support/config/my_map.yaml
====================
image: my_map.pgm
mode: trinary
resolution: 0.05
origin: [-8.66, -3.23, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25


====================
File: src/ros2_mapping_support/config/nav2_params.yaml
====================
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: "scan"
    min_particles: 500
    max_particles: 2000
    laser_min_range: 0.05
    laser_max_range: 8.0
    z_hit: 0.95
    z_rand: 0.05
    sigma_hit: 0.2
    lambda_short: 0.1
    tf_broadcast: true
    update_min_a: 0.2
    update_min_d: 0.25
    resample_interval: 1
    transform_tolerance: 0.1

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_smoother_selector_bt_node
      - nav2_planner_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 0.1
      stateful: true

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3
      min_linear_vel: 0.05
      max_linear_accel: 0.5
      max_linear_decel: 0.5
      max_angular_vel: 1.0
      lookahead_dist: 0.5
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.8
      lookahead_time: 1.5
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true
      use_angular_velocity_scaled_lookahead_dist: false
      use_rotate_to_heading: true
      rotate_to_heading_angular_vel: 0.6
      max_robot_pose_search_dist: 0.2

local_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: "odom"
    robot_base_frame: "base_link"
    update_frequency: 10.0
    publish_frequency: 10.0
    rolling_window: true
    width: 3.0
    height: 3.0
    resolution: 0.05
    robot_radius: 0.24
    footprint_padding: 0.01
    plugin_names: ["static_layer", "obstacle_layer", "inflation_layer"]
    plugin_types:
      - "nav2_costmap_2d::StaticLayer"
      - "nav2_costmap_2d::ObstacleLayer"
      - "nav2_costmap_2d::InflationLayer"

    static_layer:
      map_subscribe_transient_local: true
      subscribe_to_updates: true
      enabled: true

    obstacle_layer:
      enabled: true
      observation_sources: "scan"
      scan:
        topic: "scan"
        max_obstacle_height: 0.60
        clearing: true
        marking: true
        obstacle_range: 2.5
        raytrace_range: 3.0
        min_obstacle_height: 0.0

    inflation_layer:
      enabled: true
      inflation_radius: 0.6
      cost_scaling_factor: 3.0

global_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    update_frequency: 5.0
    publish_frequency: 2.0
    width: 10.0
    height: 10.0
    resolution: 0.05
    robot_radius: 0.24
    footprint_padding: 0.01
    rolling_window: false
    plugin_names: ["static_layer", "obstacle_layer", "inflation_layer"]
    plugin_types:
      - "nav2_costmap_2d::StaticLayer"
      - "nav2_costmap_2d::ObstacleLayer"
      - "nav2_costmap_2d::InflationLayer"

    static_layer:
      map_subscribe_transient_local: true
      subscribe_to_updates: true
      enabled: true

    obstacle_layer:
      enabled: true
      observation_sources: "scan"
      scan:
        topic: "scan"
        max_obstacle_height: 0.60
        clearing: true
        marking: true
        obstacle_range: 2.5
        raytrace_range: 3.0
        min_obstacle_height: 0.0

    inflation_layer:
      enabled: true
      inflation_radius: 0.8
      cost_scaling_factor: 3.0

planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.12
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000
      max_planning_time: 2.0
      smooth_path: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    spin_plugin: "spin"
    backup_plugin: "backup"
    wait_plugin: "wait"
    drive_on_heading_plugin: "drive_on_heading"

    spin:
      plugin: "nav2_behaviors/Spin"

    backup:
      plugin: "nav2_behaviors/BackUp"

    wait:
      plugin: "nav2_behaviors/Wait"

    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"

    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: false
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: true
    max_velocity: [0.3, 0.0, 1.0]
    min_velocity: [0.0, 0.0, 0.0]
    max_accel: [0.5, 0.0, 1.5]
    max_decel: [0.5, 0.0, 1.5]
    odom_topic: "odom"
    use_shortest_angular_distance: true

lifecycle_manager_navigation:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names:
      - "controller_server"
      - "planner_server"
      - "behavior_server"
      - "bt_navigator"
      - "waypoint_follower"
      - "velocity_smoother"


====================
File: src/ros2_mapping_support/config/nav2_params_cartographer_slam.yaml
====================
# /home/team4/carto_ws/src/ros2_mapping_support/config/nav2_params_amcl.yaml

amcl:
  ros__parameters:
    use_sim_time: false
    use_map_topic: true
    scan_topic: "scan"

    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

    base_frame_id: "imu_link"
    odom_frame_id: "map"
    global_frame_id: "map"

    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false

    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "diff"
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.1
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "$(find-pkg-share ros2_mapping_support)/config/my_map.yaml"
    frame_id: "map"
    topic_name: "map"
    use_pose: false

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.12
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      motion_model_for_search: "Dubin"
      angle_quantization_bins: 72
      minimum_turning_radius: 0.25
      cost_travel_multiplier: 2.0
      debug_visualizations: false

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.10
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.20       # bigger goal area
      yaw_goal_tolerance: 0.80      # very relaxed orientation
      stateful: true

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4
      min_linear_vel: 0.05
      max_angular_vel: 2.0
      min_angular_vel: 1.1          # higher minimum turn speed
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.8
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 2.0
      transform_tolerance: 0.2
      use_velocity_scaled_lookahead_dist: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.05
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "imu_link"
    odom_topic: "odom"
    transform_tolerance: 0.2
    default_server_timeout: 30
    wait_for_service_timeout: 5
    bt_loop_duration: 10
    goal_reached_tol: 0.05

    default_nav_to_pose_bt_xml: "$(find-pkg-share ros2_mapping_support)/config/simple_nav.xml"
    default_nav_through_poses_bt_xml: "$(find-pkg-share ros2_mapping_support)/config/simple_nav.xml"

    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_rate_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node

behavior_server:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "imu_link"
    transform_tolerance: 0.2
    progress_check_timeout: 10.0
    cycle_frequency: 10.0
    enable_stamped_cmd_vel: false

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 0.1
      max_its: 1000

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: true
    waypoint_task_executor_plugin: "wait_at_waypoint"

    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: false
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    scale_velocities: false
    max_velocity: [0.8, 0.0, 2.0]
    min_velocity: [0.0, 0.0, -2.0]

global_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "imu_link"
    update_frequency: 5.0
    publish_frequency: 2.0
    rolling_window: false
    track_unknown_space: true
    resolution: 0.05
    robot_radius: 0.30
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "scan"
      scan:
        topic: "scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.70
      cost_scaling_factor: 2.5

local_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: "odom"
    robot_base_frame: "imu_link"
    update_frequency: 10.0
    publish_frequency: 5.0
    rolling_window: true
    track_unknown_space: true
    width: 4.0
    height: 4.0
    resolution: 0.05
    robot_radius: 0.30

    # CHANGED: local_costmap only uses lidar-based obstacle_layer + inflation_layer
    plugins: ["obstacle_layer", "inflation_layer"]

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "scan"
      scan:
        topic: "scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.70
      cost_scaling_factor: 2.5


====================
File: src/ros2_mapping_support/config/nav2_params_cartographer_slam_resolved.yaml
====================
# /home/team4/carto_ws/src/ros2_mapping_support/config/nav2_params_amcl.yaml

amcl:
  ros__parameters:
    use_sim_time: false
    use_map_topic: true
    scan_topic: "scan"

    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"

    # Auto-seed AMCL pose in map frame (adjust to your real start pose).
    initial_pose_is_known: true
    initial_pose_x: 0.0
    initial_pose_y: 0.0
    initial_pose_a: 0.0    # yaw in radians
    initial_cov_xx: 0.25
    initial_cov_yy: 0.25
    initial_cov_aa: 0.1

    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false

    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.1
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "/home/team4/carto_ws/src/ros2_mapping_support/config/my_map.yaml"
    frame_id: "map"
    topic_name: "map"
    use_pose: false

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.12
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      motion_model_for_search: "Dubin"
      angle_quantization_bins: 72
      minimum_turning_radius: 0.25
      cost_travel_multiplier: 2.0
      debug_visualizations: false

controller_server:
  ros__parameters:
    __file_tag: "carto_resolved_v1"
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.10
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.20       # bigger goal area
      yaw_goal_tolerance: 0.80      # very relaxed orientation
      stateful: true

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4
      min_linear_vel: 0.05
      max_angular_vel: 2.0
      min_angular_vel: 1.1          # higher minimum turn speed
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.8
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 2.0
      transform_tolerance: 0.2
      use_velocity_scaled_lookahead_dist: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.05
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785

bt_navigator:
  ros__parameters:
    use_sim_time: false
    # Use odom here so the local costmap can mark obstacles immediately
    # even before AMCL provides a map->odom transform.
    global_frame: "odom"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    transform_tolerance: 0.2
    default_server_timeout: 30
    wait_for_service_timeout: 5
    bt_loop_duration: 10
    goal_reached_tol: 0.05

    default_nav_to_pose_bt_xml: "/home/team4/carto_ws/src/ros2_mapping_support/config/simple_nav.xml"
    default_nav_through_poses_bt_xml: "/home/team4/carto_ws/src/ros2_mapping_support/config/simple_nav.xml"

    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_rate_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node

behavior_server:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    transform_tolerance: 0.2
    progress_check_timeout: 10.0
    cycle_frequency: 10.0
    enable_stamped_cmd_vel: false

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 0.1
      max_its: 1000

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: true
    waypoint_task_executor_plugin: "wait_at_waypoint"

    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: false
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    scale_velocities: false
    max_velocity: [0.8, 0.0, 2.0]
    min_velocity: [0.0, 0.0, -2.0]

global_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    update_frequency: 15.0
    publish_frequency: 10.0
    rolling_window: false
    track_unknown_space: true
    resolution: 0.05
    robot_radius: 0.50
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      obstacle_range: 3.0
      raytrace_range: 4.0
      observation_persistence: 0.0
      expected_update_rate: 0.0
      combination_method: 1
      observation_sources: "scan"
      scan:
        topic: "/scan"
        max_obstacle_height: 1.5
        clearing: true
        marking: true
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.70
      cost_scaling_factor: 2.5

local_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    update_frequency: 20.0
    publish_frequency: 15.0
    rolling_window: true
    track_unknown_space: true
    width: 4.0
    height: 4.0
    resolution: 0.05
    robot_radius: 0.50

    # CHANGED: local_costmap only uses lidar-based obstacle_layer + inflation_layer
    plugins: ["obstacle_layer", "inflation_layer"]
    __file_tag: "carto_resolved_v1"

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      obstacle_range: 3.0
      raytrace_range: 4.0
      observation_persistence: 0.0
      expected_update_rate: 0.0
      combination_method: 1
      observation_sources: "scan"
      scan:
        topic: "/scan"
        max_obstacle_height: 1.5
        clearing: true
        marking: true
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.70
      cost_scaling_factor: 2.5


====================
File: src/ros2_mapping_support/config/simple_nav.xml
====================
<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="MainTree">

  <BehaviorTree ID="MainTree">
    <Sequence name="navigate_sequence">
      <ComputePathToPose
        name="compute_path_to_pose"
        goal="{goal}"
        path="{path}"
        planner_id="GridBased"/>

      <FollowPath
        name="follow_path"
        path="{path}"
        controller_id="FollowPath"/>
    </Sequence>
  </BehaviorTree>

</root>


====================
File: src/ros2_mapping_support/scripts/bno055_save_offsets.py
====================
#!/usr/bin/env python3
import time
import json
import os
import smbus

# ==== EDIT THESE IF NEEDED ====
I2C_BUS   = 7
I2C_ADDR  = 0x28
CALIB_FILE = os.path.expanduser("~/.ros/bno055_offsets.json")
# ===============================

# BNO055 registers / constants
BNO055_OPR_MODE        = 0x3D
BNO055_SYS_TRIGGER     = 0x3F
BNO055_POWER_MODE      = 0x3E

BNO055_UNIT_SEL        = 0x3B
BNO055_PAGE_ID         = 0x07

BNO055_CALIB_STAT      = 0x35

# Operation modes
OPR_MODE_CONFIG        = 0x00
OPR_MODE_NDOF          = 0x0C  # gyro+acc+mag fusion

# Offset registers start (acc, mag, gyro, radii)
BNO055_OFFSET_START    = 0x55
BNO055_OFFSET_END      = 0x6A  # inclusive


def write8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val & 0xFF)


def read8(bus, addr, reg):
    return bus.read_byte_data(addr, reg) & 0xFF


def set_mode(bus, addr, mode):
    write8(bus, addr, BNO055_OPR_MODE, mode)
    time.sleep(0.03)


def set_units_and_power(bus, addr):
    # Use degrees, deg/s, m/s^2, Celsius, Android orientation
    # UNIT_SEL: 0b0000_0000 => default, but we set explicitly
    write8(bus, addr, BNO055_UNIT_SEL, 0x00)
    # Normal power mode
    write8(bus, addr, BNO055_POWER_MODE, 0x00)
    # Ensure page 0
    write8(bus, addr, BNO055_PAGE_ID, 0x00)
    time.sleep(0.05)


def get_calib_status(bus, addr):
    stat = read8(bus, addr, BNO055_CALIB_STAT)
    sys  = (stat >> 6) & 0x03
    gyro = (stat >> 4) & 0x03
    accel= (stat >> 2) & 0x03
    mag  =  stat       & 0x03
    return sys, gyro, accel, mag


def read_offsets(bus, addr):
    # BNO055 offset block is 22 bytes (0x55–0x6A)
    length = BNO055_OFFSET_END - BNO055_OFFSET_START + 1
    raw = bus.read_i2c_block_data(addr, BNO055_OFFSET_START, length)

    def s16(lo, hi):
        v = (hi << 8) | lo
        if v & 0x8000:
            v -= 0x10000
        return v

    offs = {}
    # Order from datasheet:
    # ACC_OFFSET_X_LSB (0x55), ACC_OFFSET_X_MSB (0x56), ...
    offs["acc_offset_x"] = s16(raw[0],  raw[1])
    offs["acc_offset_y"] = s16(raw[2],  raw[3])
    offs["acc_offset_z"] = s16(raw[4],  raw[5])

    offs["mag_offset_x"] = s16(raw[6],  raw[7])
    offs["mag_offset_y"] = s16(raw[8],  raw[9])
    offs["mag_offset_z"] = s16(raw[10], raw[11])

    offs["gyro_offset_x"] = s16(raw[12], raw[13])
    offs["gyro_offset_y"] = s16(raw[14], raw[15])
    offs["gyro_offset_z"] = s16(raw[16], raw[17])

    offs["acc_radius"] = s16(raw[18], raw[19])
    offs["mag_radius"] = s16(raw[20], raw[21])

    return offs


def main():
    bus = smbus.SMBus(I2C_BUS)

    print("Putting BNO055 into CONFIG mode...")
    set_mode(bus, I2C_ADDR, OPR_MODE_CONFIG)
    set_units_and_power(bus, I2C_ADDR)

    print("Switching to NDOF mode (gyro+acc+mag fusion)...")
    set_mode(bus, I2C_ADDR, OPR_MODE_NDOF)

    print("Move the IMU slowly to calibrate:")
    print("  - Leave flat & still for gyro.")
    print("  - Tilt around X/Y for accel.")
    print("  - Do slow figure-8 for mag.")
    print("Waiting for calib: sys>=3, gyro=3, accel=3, mag=3 ...")

    while True:
        sys, gyro, accel, mag = get_calib_status(bus, I2C_ADDR)
        print(f"Calib status -> sys:{sys} gyro:{gyro} accel:{accel} mag:{mag}")
        if sys >= 3 and gyro == 3 and accel == 3 and mag == 3:
            print("All calibrated (3/3/3/3).")
            break
        time.sleep(1.0)

    print("Switch back to CONFIG mode to read offsets...")
    set_mode(bus, I2C_ADDR, OPR_MODE_CONFIG)

    offs = read_offsets(bus, I2C_ADDR)
    print("Read offsets:")
    for k, v in offs.items():
        print(f"  {k}: {v}")

    os.makedirs(os.path.dirname(CALIB_FILE), exist_ok=True)
    with open(CALIB_FILE, "w") as f:
        json.dump(offs, f, indent=2)

    print(f"\nSaved offsets to: {CALIB_FILE}")
    print("You only need to run this script occasionally (after big changes).")


if __name__ == "__main__":
    main()
