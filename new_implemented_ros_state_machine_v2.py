
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
