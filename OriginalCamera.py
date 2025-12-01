#!/usr/bin/env python3
import os, time, collections, cv2, numpy as np,smbus
from ultralytics import YOLO
import Jetson.GPIO as GPIO
from Focuser import Focuser  # assuming you saved your previous class
import math


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
