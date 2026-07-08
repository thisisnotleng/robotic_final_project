from flask import Flask, Response
import cv2
import numpy as np
import os
import serial
import threading
import time

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


# ----------------------------
# Robot / serial configuration
# ----------------------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Servo angles
SERVO_DEFAULT_ANGLE = 0
SERVO_OBSTACLE_ANGLE = 15

SERVO_RIGHT_SCAN_ANGLE = SERVO_DEFAULT_ANGLE
SERVO_LEFT_SCAN_ANGLE = SERVO_DEFAULT_ANGLE

# Normal movement tuning
ROBOT_SPEED = 8
MIN_FOLLOW_SPEED = 5
MAX_FOLLOW_SPEED = 25

# Stronger turn speed for curves.
LINE_TURN_SPEED = 19

# Recovery turn must not be weak.
RECOVERY_TURN_SPEED = 20

COMMAND_SEND_INTERVAL = 0.02
SAME_COMMAND_INTERVAL = 0.05


# ----------------------------
# Lost-line recovery behavior
# ----------------------------
LOST_LINE_START_SECONDS = 0.12

RECOVERY_SEARCH_TURN_SECONDS = 1.15
RECOVERY_RETURN_TURN_SECONDS = 1.08
RECOVERY_CHECK_SECONDS = 0.20
RECOVERY_REPEAT_SCAN = True


# ----------------------------
# Smoothness tuning
# ----------------------------
ERROR_SMOOTHING_ALPHA = 0.8
TURN_RELEASE_RATIO = 0.7
RECOVERY_EXIT_FOUND_FRAMES = 3
SERVO_MOVE_MIN_INTERVAL = 0.5
CENTER_OFFSET_PX = 0


# ----------------------------
# Obstacle detection
# ----------------------------
OBSTACLE_STOP_CM = 40.0
OBSTACLE_CLEAR_EXIT_CM = 55.0

OBSTACLE_CONFIRM_READINGS = 2
ULTRASONIC_STALE_SECONDS = 1.0

OBSTACLE_SCAN_BOX_TIMEOUT_SECONDS = 1.20

YOLO_MODEL_PATHS = [
    os.path.join(os.path.dirname(__file__), "best.onnx"),
    os.path.join(os.getcwd(), "best.onnx"),
]
BOX_CONFIDENCE_THRESHOLD = 0.45
OBSTACLE_SERVO_SETTLE_SECONDS = 0.45

# Briefly rotate toward the detected box before running the timed overpass path.
OBSTACLE_ALIGN_ENABLED = True
OBSTACLE_ALIGN_TOLERANCE_PX = 55
OBSTACLE_ALIGN_SPEED = 20
OBSTACLE_ALIGN_TIMEOUT_SECONDS = 0.75


# ============================================================
# SEPARATE TUNABLE OBSTACLE OVERPASS CONTROLLER
# ============================================================
# This section is completely separate from normal line following.
#
# It does NOT use:
# - ROBOT_SPEED
# - LINE_TURN_SPEED
# - RECOVERY_TURN_SPEED
#
# Green box = overpass LEFT.
# Red box   = overpass RIGHT.
#
# Each step:
# {
#     "name": debug name,
#     "command": "F" / "Q" / "E" / "S",
#     "speed": speed value,
#     "seconds": duration,
#     "check_line": True/False,
# }
#
# IMPORTANT:
# - check_line False = ignore line detection during escape/bypass.
# - check_line True  = robot starts looking for line again.
# ============================================================

OVERPASS_BRAKE_SECONDS = 0.08
OVERPASS_EXIT_FOUND_FRAMES = 2
OVERPASS_BACK_IN_TOTAL_TIMEOUT_SECONDS = 5.00

# Green box = move out LEFT, then come back RIGHT.
LEFT_OVERPASS_STEPS = [
    {
        "name": "LEFT: rotate out left",
        "command": "Q",
        "speed": 30,
        "seconds": 0.6,
        "check_line": False,
    },
    {
        "name": "LEFT: forward out from line",
        "command": "F",
        "speed": 20,
        "seconds": 1.2,
        "check_line": False,
    },
    {
        "name": "LEFT: rotate right to parallel",
        "command": "E",
        "speed": 28,
        "seconds": 0.7,
        "check_line": False,
    },
    {
        "name": "LEFT: forward beside obstacle",
        "command": "F",
        "speed": 17,
        "seconds": 1.8,
        "check_line": False,
    },
    {
        "name": "LEFT: turn right back to line",
        "command": "E",
        "speed": 22,
        "seconds": 0.16,
        "check_line": True,
    },
    {
        "name": "LEFT: forward search line",
        "command": "F",
        "speed": 13,
        "seconds": 0.18,
        "check_line": True,
    },
]

# Red box = move out RIGHT, then come back LEFT.
# Right turn uses stronger speed because your robot's right turn is weaker.
RIGHT_OVERPASS_STEPS = [
    {
        "name": "RIGHT: rotate out right",
        "command": "E",
        "speed": 30,
        "seconds": 0.6,
        "check_line": False,
    },
    {
        "name": "RIGHT: forward out from line",
        "command": "F",
        "speed": 17,
        "seconds": 0.60,
        "check_line": False,
    },
    {
        "name": "RIGHT: rotate left to parallel",
        "command": "Q",
        "speed": 28,
        "seconds": 0.30,
        "check_line": False,
    },
    {
        "name": "RIGHT: forward beside obstacle",
        "command": "F",
        "speed": 17,
        "seconds": 1.25,
        "check_line": False,
    },
    {
        "name": "RIGHT: turn left back to line",
        "command": "Q",
        "speed": 22,
        "seconds": 0.16,
        "check_line": True,
    },
    {
        "name": "RIGHT: forward search line",
        "command": "F",
        "speed": 13,
        "seconds": 0.18,
        "check_line": True,
    },
]


# ----------------------------
# Camera configuration
# ----------------------------
CAMERA_INDEX = 0

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30

DETECT_WIDTH = 320
DETECT_HEIGHT = 240

CAMERA_REOPEN_DELAY_SECONDS = 1.0
CAMERA_UNAVAILABLE_LOG_INTERVAL = 5.0
JPEG_QUALITY = 78


# ----------------------------
# Line detection configuration
# ----------------------------
BLACK_THRESHOLD = 135
BLACK_LOCAL_CONTRAST = 10
BLACK_STRONG_THRESHOLD = 90

MIN_LINE_AREA_BOTTOM = 70
MIN_LINE_AREA_LOOKAHEAD = 45

BOTTOM_ROI_TOP_RATIO = 0.70
BOTTOM_ROI_BOTTOM_RATIO = 0.98

LOOKAHEAD_ROI_TOP_RATIO = 0.45
LOOKAHEAD_ROI_BOTTOM_RATIO = 0.70

CENTER_TOLERANCE = 50

LOOKAHEAD_WEIGHT = 1.35
BOTTOM_WEIGHT = 1.00

CROP_LEFT_RATIO = 0.00
CROP_RIGHT_RATIO = 1.00

USE_OTSU_THRESHOLD = False


app = Flask(__name__)

latest_frame = None
latest_frame_seq = 0
latest_frame_lock = threading.Lock()
running = True


class RobotSerial:
    def __init__(self, port, baud_rate):
        self.ser = None
        self.last_payload = None
        self.last_sent_at = 0.0
        self.lock = threading.Lock()

        self.rx_buffer = ""
        self.last_distance_cm = None
        self.last_distance_at = 0.0

        try:
            self.ser = serial.Serial(port, baud_rate, timeout=0.05)
            time.sleep(2)
            print(f"Connected to robot on {port}")

            self.send_servo(SERVO_DEFAULT_ANGLE, force=True)
            print(f"Servo default angle set to {SERVO_DEFAULT_ANGLE}")

        except serial.SerialException as exc:
            print(f"Serial not connected: {exc}")
            print("Video server will run, but robot commands will not be sent.")

    def send(self, command, speed=ROBOT_SPEED, force=False):
        if command == "S":
            payload = "S\n"
        else:
            payload = f"{command}{int(speed)}\n"

        now = time.time()
        min_interval = SAME_COMMAND_INTERVAL if payload == self.last_payload else COMMAND_SEND_INTERVAL

        if not force and now - self.last_sent_at < min_interval:
            return

        if not self._write_payload(payload):
            return

        self.last_payload = payload
        self.last_sent_at = now
        print("Sent:", payload.strip())

    def send_servo(self, angle, force=False):
        angle = int(max(0, min(180, angle)))
        self.send("A", angle, force=force)

    def poll_responses(self):
        if not self.ser or not self.ser.is_open:
            return

        try:
            with self.lock:
                waiting = self.ser.in_waiting
                data = self.ser.read(waiting) if waiting > 0 else b""
        except (serial.SerialException, OSError) as exc:
            print(f"Serial read failed: {exc}")
            return

        if not data:
            return

        self.rx_buffer += data.decode("utf-8", errors="ignore")

        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.strip()

            if line.startswith("DIST:"):
                try:
                    self.last_distance_cm = float(line[5:])
                    self.last_distance_at = time.time()
                except ValueError:
                    pass

    def get_distance(self):
        if self.last_distance_cm is None:
            return None, 0.0

        if time.time() - self.last_distance_at > ULTRASONIC_STALE_SECONDS:
            return None, 0.0

        return self.last_distance_cm, self.last_distance_at

    def _write_payload(self, payload):
        if not self.ser or not self.ser.is_open:
            print(f"Serial not connected; could not send {payload.strip()}")
            return False

        try:
            with self.lock:
                self.ser.write(payload.encode("utf-8"))
        except serial.SerialException as exc:
            print(f"Serial write failed: {exc}")

            try:
                self.ser.close()
            except serial.SerialException:
                pass

            self.ser = None
            self.last_payload = None
            return False

        return True

    def close(self):
        self.send("S", force=True)

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException as exc:
                print(f"Serial close failed: {exc}")


robot = RobotSerial(SERIAL_PORT, BAUD_RATE)

box_model = None
box_model_path = next((path for path in YOLO_MODEL_PATHS if os.path.exists(path)), None)

if YOLO is None:
    print("ultralytics is not installed; obstacle box classification is disabled.")
elif box_model_path is None:
    print(f"YOLO model not found at {YOLO_MODEL_PATHS}; obstacle box classification is disabled.")
else:
    try:
        box_model = YOLO(box_model_path)
        print(f"Loaded YOLO model: {box_model_path}")
    except Exception as exc:
        print(f"Could not load YOLO model {box_model_path}: {exc}")


# ----------------------------
# State
# ----------------------------
last_line_seen_at = time.time()
recovery_mode = False
recovery_phase = "NORMAL"
recovery_phase_started_at = 0.0
servo_angle_now = SERVO_DEFAULT_ANGLE
servo_last_moved_at = 0.0
recovery_found_streak = 0
smoothed_error = 0.0
last_follow_action = None

# Obstacle state
obstacle_mode = False
obstacle_phase = "CLEAR"
obstacle_phase_started_at = 0.0
obstacle_below_count = 0
obstacle_last_reading_at = 0.0
obstacle_box_label = None
obstacle_box_conf = 0.0
obstacle_overpass_side = None
obstacle_align_label = None
obstacle_align_conf = 0.0
obstacle_align_error = 0
overpass_found_line_streak = 0
latest_box_detections = []

# Tunable overpass internal state
overpass_brake_until = 0.0
overpass_back_in_started_at = 0.0
overpass_steps = []
overpass_step_index = 0


def set_recovery_phase(phase):
    global recovery_phase, recovery_phase_started_at
    recovery_phase = phase
    recovery_phase_started_at = time.time()
    print(f"Recovery phase: {phase}")


def set_servo_angle(angle, force=False):
    global servo_angle_now, servo_last_moved_at

    if servo_angle_now == angle and not force:
        return

    now = time.time()
    if not force and now - servo_last_moved_at < SERVO_MOVE_MIN_INTERVAL:
        return

    servo_angle_now = angle
    servo_last_moved_at = now
    robot.send_servo(angle, force=True)


def set_obstacle_phase(phase):
    global obstacle_phase, obstacle_phase_started_at
    global overpass_brake_until

    obstacle_phase = phase
    obstacle_phase_started_at = time.time()

    if phase == "OVERPASS_RUN":
        overpass_brake_until = time.time() + OVERPASS_BRAKE_SECONDS
    else:
        overpass_brake_until = 0.0

    print(f"Obstacle phase: {phase}")


def start_recovery():
    global recovery_mode, recovery_found_streak, last_follow_action

    recovery_mode = True
    recovery_found_streak = 0
    last_follow_action = None

    robot.send("S", force=True)
    set_servo_angle(SERVO_DEFAULT_ANGLE)
    set_recovery_phase("SEARCH_LEFT")


def stop_recovery():
    global recovery_mode, recovery_found_streak, smoothed_error

    recovery_mode = False
    recovery_found_streak = 0
    smoothed_error = 0.0
    set_servo_angle(SERVO_DEFAULT_ANGLE)
    set_recovery_phase("NORMAL")


# ----------------------------
# Obstacle handling
# ----------------------------
def enter_obstacle_mode():
    global obstacle_mode, recovery_mode, last_follow_action
    global obstacle_box_label, obstacle_box_conf, obstacle_overpass_side
    global obstacle_align_label, obstacle_align_conf, obstacle_align_error
    global overpass_found_line_streak, latest_box_detections
    global overpass_brake_until, overpass_back_in_started_at
    global overpass_steps, overpass_step_index

    obstacle_mode = True
    last_follow_action = None
    obstacle_box_label = None
    obstacle_box_conf = 0.0
    obstacle_overpass_side = None
    obstacle_align_label = None
    obstacle_align_conf = 0.0
    obstacle_align_error = 0
    overpass_found_line_streak = 0
    latest_box_detections = []

    overpass_brake_until = 0.0
    overpass_back_in_started_at = 0.0
    overpass_steps = []
    overpass_step_index = 0

    if recovery_mode:
        recovery_mode = False
        set_recovery_phase("NORMAL")

    robot.send("S", force=True)
    set_servo_angle(SERVO_OBSTACLE_ANGLE, force=True)
    set_obstacle_phase("SCAN_SETTLE")
    print("Obstacle detected: robot stopped, servo moved for box scan.")


def exit_obstacle_mode():
    global obstacle_mode, obstacle_below_count, last_line_seen_at
    global obstacle_box_label, obstacle_box_conf, obstacle_overpass_side
    global obstacle_align_label, obstacle_align_conf, obstacle_align_error
    global overpass_found_line_streak, latest_box_detections
    global overpass_brake_until, overpass_back_in_started_at
    global overpass_steps, overpass_step_index
    global smoothed_error, last_follow_action

    obstacle_mode = False
    obstacle_below_count = 0
    obstacle_box_label = None
    obstacle_box_conf = 0.0
    obstacle_overpass_side = None
    obstacle_align_label = None
    obstacle_align_conf = 0.0
    obstacle_align_error = 0
    overpass_found_line_streak = 0
    latest_box_detections = []

    overpass_brake_until = 0.0
    overpass_back_in_started_at = 0.0
    overpass_steps = []
    overpass_step_index = 0

    smoothed_error = 0.0
    last_follow_action = None

    set_obstacle_phase("CLEAR")
    set_servo_angle(SERVO_DEFAULT_ANGLE, force=True)

    last_line_seen_at = time.time()
    print("Obstacle handling complete: resuming line following.")


def update_obstacle_state():
    global obstacle_below_count, obstacle_last_reading_at

    distance, measured_at = robot.get_distance()

    if distance is None or measured_at <= obstacle_last_reading_at:
        return

    obstacle_last_reading_at = measured_at

    blocked = 0 < distance <= OBSTACLE_STOP_CM

    if not obstacle_mode:
        if blocked:
            obstacle_below_count += 1

            if obstacle_below_count >= OBSTACLE_CONFIRM_READINGS:
                enter_obstacle_mode()
        else:
            obstacle_below_count = 0

        return


# ----------------------------
# Vision helpers
# ----------------------------
def create_black_mask(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    local_mean = cv2.GaussianBlur(gray, (31, 31), 0)
    local_contrast = cv2.subtract(local_mean, gray)

    if USE_OTSU_THRESHOLD:
        _, mask = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
        )
    else:
        dark_mask = cv2.inRange(gray, 0, BLACK_THRESHOLD)
        mask = dark_mask

    contrast_mask = cv2.inRange(local_contrast, BLACK_LOCAL_CONTRAST, 255)
    strong_dark_mask = cv2.inRange(gray, 0, BLACK_STRONG_THRESHOLD)
    line_like_mask = cv2.bitwise_or(contrast_mask, strong_dark_mask)
    mask = cv2.bitwise_and(mask, line_like_mask)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def find_line_center_in_roi(frame_small, top_ratio, bottom_ratio, min_area):
    h, w = frame_small.shape[:2]

    crop_left = int(CROP_LEFT_RATIO * w)
    crop_right = int(CROP_RIGHT_RATIO * w)

    roi_top = int(top_ratio * h)
    roi_bottom = int(bottom_ratio * h)

    roi = frame_small[roi_top:roi_bottom, crop_left:crop_right]
    mask = create_black_mask(roi)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if not contours:
        return {
            "found": False,
            "center_x": None,
            "center_y": None,
            "area": 0,
            "roi_top": roi_top,
            "roi_bottom": roi_bottom,
            "crop_left": crop_left,
            "crop_right": crop_right,
            "mask": mask,
        }

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < min_area:
        return {
            "found": False,
            "center_x": None,
            "center_y": None,
            "area": area,
            "roi_top": roi_top,
            "roi_bottom": roi_bottom,
            "crop_left": crop_left,
            "crop_right": crop_right,
            "mask": mask,
        }

    M = cv2.moments(largest)

    if M["m00"] <= 0:
        return {
            "found": False,
            "center_x": None,
            "center_y": None,
            "area": area,
            "roi_top": roi_top,
            "roi_bottom": roi_bottom,
            "crop_left": crop_left,
            "crop_right": crop_right,
            "mask": mask,
        }

    local_center_x = int(M["m10"] / M["m00"])
    local_center_y = int(M["m01"] / M["m00"])

    return {
        "found": True,
        "center_x": crop_left + local_center_x,
        "center_y": roi_top + local_center_y,
        "area": area,
        "roi_top": roi_top,
        "roi_bottom": roi_bottom,
        "crop_left": crop_left,
        "crop_right": crop_right,
        "mask": mask,
    }


def detect_line(frame):
    frame_small = cv2.resize(
        frame,
        (DETECT_WIDTH, DETECT_HEIGHT),
        interpolation=cv2.INTER_AREA,
    )

    bottom = find_line_center_in_roi(
        frame_small,
        BOTTOM_ROI_TOP_RATIO,
        BOTTOM_ROI_BOTTOM_RATIO,
        MIN_LINE_AREA_BOTTOM,
    )

    lookahead = find_line_center_in_roi(
        frame_small,
        LOOKAHEAD_ROI_TOP_RATIO,
        LOOKAHEAD_ROI_BOTTOM_RATIO,
        MIN_LINE_AREA_LOOKAHEAD,
    )

    found_line = bottom["found"] or lookahead["found"]

    h, w = frame_small.shape[:2]
    reference_x = (w // 2) + CENTER_OFFSET_PX

    bottom_error = 0
    lookahead_error = 0

    if bottom["found"]:
        bottom_error = bottom["center_x"] - reference_x

    if lookahead["found"]:
        lookahead_error = lookahead["center_x"] - reference_x

    weighted_error = 0
    total_weight = 0

    if bottom["found"]:
        weighted_error += bottom_error * BOTTOM_WEIGHT
        total_weight += BOTTOM_WEIGHT

    if lookahead["found"]:
        weighted_error += lookahead_error * LOOKAHEAD_WEIGHT
        total_weight += LOOKAHEAD_WEIGHT

    if total_weight > 0:
        weighted_error = weighted_error / total_weight

    scale_x = FRAME_WIDTH / DETECT_WIDTH
    scale_y = FRAME_HEIGHT / DETECT_HEIGHT

    return {
        "found_line": found_line,
        "bottom": bottom,
        "lookahead": lookahead,
        "reference_x": reference_x,
        "bottom_error": bottom_error,
        "lookahead_error": lookahead_error,
        "weighted_error": weighted_error,
        "frame_small_width": w,
        "frame_small_height": h,
        "scale_x": scale_x,
        "scale_y": scale_y,
    }


def detect_obstacle_box(frame):
    global latest_box_detections

    latest_box_detections = []

    if box_model is None:
        return None, 0.0

    try:
        results = box_model(frame, verbose=False)
    except Exception as exc:
        print(f"YOLO inference failed: {exc}")
        return None, 0.0

    best_label = None
    best_conf = 0.0

    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if conf < BOX_CONFIDENCE_THRESHOLD:
                continue

            raw_label = box_model.names[cls_id]
            label = str(raw_label).lower()

            if "red" in label:
                normalized_label = "redbox"
            elif "green" in label:
                normalized_label = "greenbox"
            else:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            latest_box_detections.append(
                {
                    "label": normalized_label,
                    "conf": conf,
                    "xyxy": (x1, y1, x2, y2),
                }
            )

            if conf > best_conf:
                best_label = normalized_label
                best_conf = conf

    return best_label, best_conf


def get_best_box_alignment(frame):
    if not latest_box_detections:
        return None

    best_box = max(latest_box_detections, key=lambda item: item["conf"])
    x1, _y1, x2, _y2 = best_box["xyxy"]
    box_center_x = (x1 + x2) / 2.0
    frame_center_x = frame.shape[1] / 2.0

    return {
        "label": best_box["label"],
        "conf": best_box["conf"],
        "error": box_center_x - frame_center_x,
    }


# ----------------------------
# Decision logic
# ----------------------------
def choose_follow_command(detection):
    global smoothed_error, last_follow_action

    smoothed_error = (
        ERROR_SMOOTHING_ALPHA * detection["weighted_error"]
        + (1.0 - ERROR_SMOOTHING_ALPHA) * smoothed_error
    )

    error = smoothed_error
    abs_error = abs(error)

    was_turning = last_follow_action is not None and last_follow_action[1] in ("Q", "E")

    if was_turning:
        forward_tolerance = CENTER_TOLERANCE * TURN_RELEASE_RATIO
    else:
        forward_tolerance = CENTER_TOLERANCE

    # Early curve reaction.
    if detection["lookahead"]["found"]:
        lookahead_error = detection["lookahead_error"]
        abs_lookahead_error = abs(lookahead_error)

        if abs_lookahead_error > CENTER_TOLERANCE:
            turn_speed = LINE_TURN_SPEED

            if abs_lookahead_error > 90:
                turn_speed += 7
            elif abs_lookahead_error > 65:
                turn_speed += 5
            elif abs_lookahead_error > 45:
                turn_speed += 3

            turn_speed = max(20, min(28, turn_speed))

            if lookahead_error < 0:
                last_follow_action = ("EARLY CURVE LEFT", "Q", turn_speed)
            else:
                last_follow_action = ("EARLY CURVE RIGHT", "E", turn_speed)

            return last_follow_action

    if abs_error <= forward_tolerance:
        decision = "FORWARD"
        command = "F"
        speed = ROBOT_SPEED

        last_follow_action = (decision, command, speed)
        return last_follow_action

    turn_speed = LINE_TURN_SPEED

    if abs_error > 90:
        turn_speed += 11
    elif abs_error > 65:
        turn_speed += 9
    elif abs_error > 45:
        turn_speed += 3

    turn_speed = max(20, min(28, turn_speed))

    if error < 0:
        last_follow_action = ("PIVOT LEFT", "Q", turn_speed)
    else:
        last_follow_action = ("PIVOT RIGHT", "E", turn_speed)

    return last_follow_action


def choose_recovery_command(detection):
    global last_line_seen_at, recovery_found_streak

    now = time.time()
    elapsed = now - recovery_phase_started_at

    if detection["found_line"]:
        last_line_seen_at = now
        recovery_found_streak += 1

        if recovery_found_streak >= RECOVERY_EXIT_FOUND_FRAMES:
            stop_recovery()
            return choose_follow_command(detection)

        return "RECOVERY CONFIRM LINE", "S", 0

    recovery_found_streak = 0

    if recovery_phase == "SEARCH_LEFT":
        if elapsed < RECOVERY_SEARCH_TURN_SECONDS:
            return "RECOVERY SEARCH LEFT", "Q", RECOVERY_TURN_SPEED

        robot.send("S", force=True)
        set_recovery_phase("CHECK_LEFT")
        return "RECOVERY CHECK LEFT", "S", 0

    if recovery_phase == "CHECK_LEFT":
        if elapsed < RECOVERY_CHECK_SECONDS:
            return "RECOVERY CHECK LEFT", "S", 0

        set_recovery_phase("RETURN_CENTER_FROM_LEFT")
        return "RECOVERY RETURN CENTER", "S", 0

    if recovery_phase == "RETURN_CENTER_FROM_LEFT":
        if elapsed < RECOVERY_RETURN_TURN_SECONDS:
            return "RECOVERY RETURN CENTER FROM LEFT", "E", RECOVERY_TURN_SPEED

        robot.send("S", force=True)
        set_recovery_phase("CHECK_CENTER_1")
        return "RECOVERY CHECK CENTER", "S", 0

    if recovery_phase == "CHECK_CENTER_1":
        if elapsed < RECOVERY_CHECK_SECONDS:
            return "RECOVERY CHECK CENTER", "S", 0

        set_recovery_phase("SEARCH_RIGHT")
        return "RECOVERY PREP RIGHT", "S", 0

    if recovery_phase == "SEARCH_RIGHT":
        if elapsed < RECOVERY_SEARCH_TURN_SECONDS:
            return "RECOVERY SEARCH RIGHT", "E", RECOVERY_TURN_SPEED

        robot.send("S", force=True)
        set_recovery_phase("CHECK_RIGHT")
        return "RECOVERY CHECK RIGHT", "S", 0

    if recovery_phase == "CHECK_RIGHT":
        if elapsed < RECOVERY_CHECK_SECONDS:
            return "RECOVERY CHECK RIGHT", "S", 0

        set_recovery_phase("RETURN_CENTER_FROM_RIGHT")
        return "RECOVERY RETURN CENTER", "S", 0

    if recovery_phase == "RETURN_CENTER_FROM_RIGHT":
        if elapsed < RECOVERY_RETURN_TURN_SECONDS:
            return "RECOVERY RETURN CENTER FROM RIGHT", "Q", RECOVERY_TURN_SPEED

        robot.send("S", force=True)
        set_recovery_phase("CHECK_CENTER_2")
        return "RECOVERY CHECK CENTER", "S", 0

    if recovery_phase == "CHECK_CENTER_2":
        if elapsed < RECOVERY_CHECK_SECONDS:
            return "RECOVERY CHECK CENTER", "S", 0

        if RECOVERY_REPEAT_SCAN:
            set_recovery_phase("SEARCH_LEFT")
            return "RECOVERY REPEAT SCAN", "S", 0

        return "RECOVERY FAILED STOP", "S", 0

    set_recovery_phase("SEARCH_LEFT")
    return "RECOVERY RESET", "S", 0


# ============================================================
# TUNABLE OVERPASS FUNCTIONS - TABLE-DRIVEN PATH
# ============================================================
def overpass_begin(label, conf):
    global obstacle_box_label, obstacle_box_conf, obstacle_overpass_side
    global overpass_found_line_streak
    global overpass_steps, overpass_step_index
    global overpass_back_in_started_at

    obstacle_box_label = label
    obstacle_box_conf = conf
    overpass_found_line_streak = 0
    overpass_step_index = 0
    overpass_back_in_started_at = 0.0

    if label == "greenbox":
        obstacle_overpass_side = "LEFT"
        overpass_steps = LEFT_OVERPASS_STEPS
        print("GREEN BOX: using tunable LEFT overpass path.")
    elif label == "redbox":
        obstacle_overpass_side = "RIGHT"
        overpass_steps = RIGHT_OVERPASS_STEPS
        print("RED BOX: using tunable RIGHT overpass path.")
    else:
        obstacle_overpass_side = "LEFT"
        overpass_steps = LEFT_OVERPASS_STEPS
        print("UNKNOWN BOX: defaulting to tunable LEFT overpass path.")

    set_servo_angle(SERVO_DEFAULT_ANGLE, force=True)
    set_obstacle_phase("OVERPASS_RUN")

    if overpass_steps:
        print(f"Overpass step: {overpass_steps[0]['name']}")


def overpass_go_next_step():
    global overpass_step_index, obstacle_phase_started_at
    global overpass_back_in_started_at

    overpass_step_index += 1
    obstacle_phase_started_at = time.time()

    if overpass_step_index >= len(overpass_steps):
        # Repeat last 2 steps forever until line found or timeout.
        # These should be the back-in/search-line steps.
        overpass_step_index = max(0, len(overpass_steps) - 2)

    current_step = overpass_steps[overpass_step_index]

    if current_step.get("check_line", False) and overpass_back_in_started_at <= 0:
        overpass_back_in_started_at = time.time()

    print(f"Overpass step: {current_step['name']}")


def overpass_try_resume_line_follow(detection):
    global overpass_found_line_streak

    if detection["found_line"]:
        overpass_found_line_streak += 1

        if overpass_found_line_streak >= OVERPASS_EXIT_FOUND_FRAMES:
            exit_obstacle_mode()
            return choose_follow_command(detection)
    else:
        overpass_found_line_streak = 0

    return None


def overpass_command(detection):
    global overpass_step_index
    global obstacle_phase_started_at

    now = time.time()

    if now < overpass_brake_until:
        return "OVERPASS BRAKE", "S", 0

    if not overpass_steps:
        robot.send("S", force=True)
        exit_obstacle_mode()
        start_recovery()
        return "OVERPASS NO STEPS", "S", 0

    if overpass_step_index >= len(overpass_steps):
        overpass_step_index = len(overpass_steps) - 1

    current_step = overpass_steps[overpass_step_index]
    elapsed = now - obstacle_phase_started_at

    # Only detect line again during return/search steps.
    if current_step.get("check_line", False):
        resumed = overpass_try_resume_line_follow(detection)
        if resumed is not None:
            return resumed

    if elapsed < current_step["seconds"]:
        return (
            current_step["name"],
            current_step["command"],
            current_step["speed"],
        )

    robot.send("S", force=True)
    overpass_go_next_step()

    current_step = overpass_steps[overpass_step_index]

    return (
        current_step["name"],
        current_step["command"],
        current_step["speed"],
    )


def choose_obstacle_command(detection, frame):
    global last_line_seen_at
    global obstacle_align_label, obstacle_align_conf, obstacle_align_error

    now = time.time()
    elapsed = now - obstacle_phase_started_at

    # While obstacle handling, prevent normal lost-line recovery from triggering.
    last_line_seen_at = now

    if obstacle_phase == "SCAN_SETTLE":
        set_servo_angle(SERVO_OBSTACLE_ANGLE)

        if elapsed < OBSTACLE_SERVO_SETTLE_SECONDS:
            return "OBSTACLE SCAN SETTLE", "S", 0

        set_obstacle_phase("SCAN_BOX")
        return "OBSTACLE SCAN BOX", "S", 0

    if obstacle_phase == "SCAN_BOX":
        set_servo_angle(SERVO_OBSTACLE_ANGLE)
        label, conf = detect_obstacle_box(frame)

        if label is None:
            if elapsed >= OBSTACLE_SCAN_BOX_TIMEOUT_SECONDS:
                robot.send("S", force=True)
                exit_obstacle_mode()

                if detection["found_line"]:
                    return choose_follow_command(detection)

                start_recovery()
                return "OBSTACLE BOX TIMEOUT", "S", 0

            if box_model is None:
                return "OBSTACLE NO YOLO MODEL", "S", 0

            return "OBSTACLE LOOKING BOX", "S", 0

        alignment = get_best_box_alignment(frame)

        obstacle_align_label = label
        obstacle_align_conf = conf
        obstacle_align_error = int(alignment["error"]) if alignment is not None else 0

        if (
            OBSTACLE_ALIGN_ENABLED
            and alignment is not None
            and abs(obstacle_align_error) > OBSTACLE_ALIGN_TOLERANCE_PX
        ):
            set_obstacle_phase("ALIGN_BOX")
            return f"ALIGN BOX START {obstacle_align_error:+d}", "S", 0

        overpass_begin(label, conf)
        return f"{label.upper()} TUNABLE OVERPASS START", "S", 0

    if obstacle_phase == "ALIGN_BOX":
        set_servo_angle(SERVO_OBSTACLE_ANGLE)
        label, conf = detect_obstacle_box(frame)
        alignment = get_best_box_alignment(frame)

        if alignment is not None:
            obstacle_align_label = label
            obstacle_align_conf = conf
            obstacle_align_error = int(alignment["error"])

        should_start_overpass = (
            alignment is not None
            and abs(obstacle_align_error) <= OBSTACLE_ALIGN_TOLERANCE_PX
        )
        timed_out = elapsed >= OBSTACLE_ALIGN_TIMEOUT_SECONDS

        if should_start_overpass or timed_out:
            robot.send("S", force=True)
            overpass_begin(obstacle_align_label or label, obstacle_align_conf or conf)
            return "BOX ALIGNED OVERPASS START", "S", 0

        if obstacle_align_error < 0:
            return f"ALIGN BOX LEFT {obstacle_align_error:+d}", "Q", OBSTACLE_ALIGN_SPEED

        return f"ALIGN BOX RIGHT {obstacle_align_error:+d}", "E", OBSTACLE_ALIGN_SPEED

    if obstacle_phase == "OVERPASS_RUN":
        set_servo_angle(SERVO_DEFAULT_ANGLE)

        if (
            overpass_back_in_started_at > 0
            and now - overpass_back_in_started_at > OVERPASS_BACK_IN_TOTAL_TIMEOUT_SECONDS
        ):
            robot.send("S", force=True)
            exit_obstacle_mode()
            start_recovery()
            return "OVERPASS BACK IN TIMEOUT", "S", 0

        return overpass_command(detection)

    set_obstacle_phase("SCAN_SETTLE")
    return "OBSTACLE RESET", "S", 0


def decide_robot_action(detection, frame):
    global last_line_seen_at, last_follow_action

    now = time.time()

    if obstacle_mode:
        return choose_obstacle_command(detection, frame)

    if recovery_mode:
        return choose_recovery_command(detection)

    if detection["found_line"]:
        last_line_seen_at = now

        if servo_angle_now != SERVO_DEFAULT_ANGLE:
            set_servo_angle(SERVO_DEFAULT_ANGLE)

        return choose_follow_command(detection)

    time_since_line_seen = now - last_line_seen_at

    if time_since_line_seen >= LOST_LINE_START_SECONDS:
        start_recovery()
        return "START RECOVERY", "S", 0

    # Do not coast forward when line is lost.
    if last_follow_action is not None:
        last_follow_action = None
        return "LINE LOST WAIT", "S", 0

    return "LINE LOST WAIT", "S", 0


# ----------------------------
# Debug drawing
# ----------------------------
def draw_roi_box(frame, roi_info, scale_x, scale_y, color, label):
    x1 = int(roi_info["crop_left"] * scale_x)
    x2 = int(roi_info["crop_right"] * scale_x)
    y1 = int(roi_info["roi_top"] * scale_y)
    y2 = int(roi_info["roi_bottom"] * scale_y)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    cv2.putText(
        frame,
        label,
        (x1 + 8, y1 + 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
    )

    if roi_info["found"]:
        cx = int(roi_info["center_x"] * scale_x)
        cy = int(roi_info["center_y"] * scale_y)

        cv2.circle(frame, (cx, cy), 8, color, -1)


def draw_debug(frame, detection, decision, command, speed):
    h, w = frame.shape[:2]

    scale_x = detection["scale_x"]
    scale_y = detection["scale_y"]

    if obstacle_mode:
        color = (0, 0, 255)
    elif detection["found_line"]:
        color = (0, 255, 0)
    elif recovery_mode:
        color = (0, 255, 255)
    else:
        color = (0, 0, 255)

    center_x = int(detection["reference_x"] * scale_x)
    tolerance_full = int(CENTER_TOLERANCE * scale_x)

    cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 0), 2)
    cv2.line(frame, (center_x - tolerance_full, 0), (center_x - tolerance_full, h), (120, 120, 0), 1)
    cv2.line(frame, (center_x + tolerance_full, 0), (center_x + tolerance_full, h), (120, 120, 0), 1)

    draw_roi_box(
        frame,
        detection["lookahead"],
        scale_x,
        scale_y,
        (255, 0, 255),
        "LOOKAHEAD",
    )

    draw_roi_box(
        frame,
        detection["bottom"],
        scale_x,
        scale_y,
        (0, 255, 0),
        "BOTTOM",
    )

    cv2.putText(
        frame,
        f"{decision} | CMD {command} | SPD {int(speed)}",
        (10, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        color,
        2,
    )

    cv2.putText(
        frame,
        f"ERR W:{int(detection['weighted_error'])} B:{int(detection['bottom_error'])} L:{int(detection['lookahead_error'])}",
        (10, 64),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 0),
        2,
    )

    cv2.putText(
        frame,
        f"AREA B:{int(detection['bottom']['area'])} L:{int(detection['lookahead']['area'])} | SERVO {servo_angle_now}",
        (10, 94),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 0),
        2,
    )

    if obstacle_mode:
        mode_text = "OBSTACLE"
    elif recovery_mode:
        mode_text = "RECOVERY"
    else:
        mode_text = "FOLLOW"

    distance_cm, _ = robot.get_distance()

    if distance_cm is None:
        distance_text = "--"
    elif distance_cm <= 0:
        distance_text = "CLEAR"
    else:
        distance_text = f"{distance_cm:.0f}cm"

    cv2.putText(
        frame,
        f"MODE {mode_text} | RECOVERY {recovery_phase} | DIST {distance_text}",
        (10, 124),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
        2,
    )

    step_text = "--"
    if obstacle_mode and obstacle_phase == "OVERPASS_RUN" and overpass_steps:
        safe_index = min(overpass_step_index, len(overpass_steps) - 1)
        step_text = overpass_steps[safe_index]["name"]

    cv2.putText(
        frame,
        f"OBSTACLE {obstacle_phase} | BOX {obstacle_box_label or '--'} {obstacle_box_conf:.2f} | SIDE {obstacle_overpass_side or '--'}",
        (10, 154),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"OVERPASS STEP {overpass_step_index}: {step_text}",
        (10, 184),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (255, 255, 255),
        2,
    )

    for box in latest_box_detections:
        x1, y1, x2, y2 = box["xyxy"]
        label = box["label"]
        conf = box["conf"]
        box_color = (0, 255, 0) if label == "greenbox" else (0, 0, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        cv2.putText(
            frame,
            f"{label} {conf:.2f}",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            box_color,
            2,
        )


# ----------------------------
# Frame publishing
# ----------------------------
def publish_frame(frame):
    global latest_frame, latest_frame_seq

    ok, buffer = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )

    if ok:
        with latest_frame_lock:
            latest_frame = buffer.tobytes()
            latest_frame_seq += 1


def publish_status_frame(message):
    frame = np.zeros((240, 560, 3), dtype=np.uint8)

    cv2.putText(
        frame,
        message,
        (18, 112),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )

    publish_frame(frame)


# ----------------------------
# Camera
# ----------------------------
def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        cap.release()
        return None

    for _ in range(3):
        cap.read()

    ok, frame = cap.read()

    if ok and frame is not None:
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"Using camera index {CAMERA_INDEX}: {actual_w}x{actual_h} @ {actual_fps:.1f} FPS")
        return cap

    print(f"Camera index {CAMERA_INDEX} opened but did not return frames.")
    cap.release()
    return None


def camera_control_loop():
    global running

    cap = None
    camera_read_failed = False
    last_camera_unavailable_log_at = 0.0

    publish_status_frame("Starting camera...")

    try:
        while running:
            loop_started_at = time.time()

            if cap is None:
                cap = open_camera()

                if cap is None:
                    now = time.time()

                    if now - last_camera_unavailable_log_at >= CAMERA_UNAVAILABLE_LOG_INTERVAL:
                        print("Camera unavailable; robot stopped and retrying.")
                        robot.send("S", force=True)
                        publish_status_frame("Camera unavailable")
                        last_camera_unavailable_log_at = now

                    time.sleep(CAMERA_REOPEN_DELAY_SECONDS)
                    continue

            ok = cap.grab()
            if not ok:
                frame = None
            else:
                ok, frame = cap.retrieve()

            if not ok or frame is None:
                if not camera_read_failed:
                    print("Camera read failed; stopping robot until frames return.")
                    robot.send("S", force=True)
                    publish_status_frame("Camera read failed")
                    camera_read_failed = True

                cap.release()
                cap = None
                time.sleep(CAMERA_REOPEN_DELAY_SECONDS)
                continue

            if camera_read_failed:
                print("Camera frames returned.")
                camera_read_failed = False

            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA)

            robot.poll_responses()
            update_obstacle_state()

            detection = detect_line(frame)
            decision, command, speed = decide_robot_action(detection, frame)

            robot.send(command, speed)

            draw_debug(frame, detection, decision, command, speed)

            elapsed_ms = (time.time() - loop_started_at) * 1000.0
            cv2.putText(
                frame,
                f"LOOP {elapsed_ms:.1f}ms",
                (10, FRAME_HEIGHT - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

            publish_frame(frame)

    finally:
        robot.close()

        if cap is not None:
            cap.release()


def generate_frames():
    sent_frame_seq = -1

    while running:
        with latest_frame_lock:
            frame = latest_frame
            frame_seq = latest_frame_seq

        if frame is None or frame_seq == sent_frame_seq:
            time.sleep(0.005)
            continue

        sent_frame_seq = frame_seq

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/")
def index():
    return """
<!doctype html>
<html>
<head>
    <title>Robot Line Detection</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            margin: 0;
            background: #111;
            color: white;
            font-family: Arial, sans-serif;
            text-align: center;
        }

        h1 {
            font-size: 22px;
            margin: 16px 0;
        }

        img {
            width: 100%;
            max-width: 720px;
            height: auto;
            border: 2px solid #333;
            background: black;
        }

        .hint {
            color: #bbb;
            font-size: 14px;
            margin: 10px auto 18px;
            max-width: 760px;
            padding: 0 14px;
        }
    </style>
</head>
<body>
    <h1>Robot Line Detection</h1>
    <div class="hint">
        Capture 640x480, detect 320x240. Tunable green/red obstacle overpass controller.
    </div>
    <img src="/video_feed" alt="Robot camera stream">
</body>
</html>
"""


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    worker = threading.Thread(target=camera_control_loop, daemon=True)
    worker.start()

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        running = False
        robot.close()
