from flask import Flask, Response
import cv2
import numpy as np
import serial
import threading
import time


# ----------------------------
# Robot / serial configuration
# ----------------------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Servo angles
# Camera looks down at the line by default. It only lifts to
# SERVO_OBSTACLE_ANGLE while an obstacle is blocking the path.
SERVO_DEFAULT_ANGLE = 0
SERVO_OBSTACLE_ANGLE = 15

# Recovery no longer tilts the camera; keeping both scan angles at the
# default means the servo stays still while searching for the line.
SERVO_RIGHT_SCAN_ANGLE = SERVO_DEFAULT_ANGLE
SERVO_LEFT_SCAN_ANGLE = SERVO_DEFAULT_ANGLE

# Normal movement tuning
ROBOT_SPEED = 17
MIN_FOLLOW_SPEED = 10
MAX_FOLLOW_SPEED = 25

# Pivot/turn speed while following line.
# Keep this gentle; the camera view shows pure pivots can overshoot the
# center quickly and throw the line out of frame.
LINE_TURN_SPEED = 15

# Lost-line recovery turn speed.
# Keep recovery slower than before so it can reacquire the line without
# sweeping past it too quickly.
RECOVERY_TURN_SPEED = 18

# Send serial commands fast enough for live line following
COMMAND_SEND_INTERVAL = 0.02
SAME_COMMAND_INTERVAL = 0.05


# ----------------------------
# Lost-line recovery behavior
# ----------------------------
# Wait a bit before declaring the line lost, so short detection
# dropouts do not snap the servo and restart recovery over and over.
LOST_LINE_START_SECONDS = 0.35

# Reasonable recovery turn duration.
# Increase if it does not rotate enough.
# Decrease if it rotates too much.
RECOVERY_TURN_SECONDS = 0.50

# Stop and let camera check.
RECOVERY_CHECK_SECONDS = 0.80


# ----------------------------
# Smoothness tuning
# ----------------------------
# Blend factor for the steering error (0..1).
# Lower = smoother steering, higher = faster reaction.
ERROR_SMOOTHING_ALPHA = 0.6

# While pivoting, only switch back to FORWARD once the error is within
# CENTER_TOLERANCE * TURN_RELEASE_RATIO.
# This hysteresis stops the rapid F/Q/E flickering right at the
# tolerance edge.
TURN_RELEASE_RATIO = 0.6

# Line must be visible this many consecutive frames before leaving
# recovery, so one noisy frame does not flap the mode and the servo.
RECOVERY_EXIT_FOUND_FRAMES = 3

# Minimum seconds between physical servo moves.
# Stops the servo from being hammered with rapid angle changes.
SERVO_MOVE_MIN_INTERVAL = 0.5

# Shift the steering reference (the yellow center line) left/right,
# in detection pixels (at DETECT_WIDTH=320).
# Use this if the camera is mounted slightly off-center and the center
# line does not sit on the ground line even when the robot is centered.
# Negative = move reference left, positive = move right.
CENTER_OFFSET_PX = 0


# ----------------------------
# Obstacle detection (ultrasonic)
# ----------------------------
# The firmware pushes "DIST:<cm>" lines automatically (~every 60ms).

# Stop when an obstacle is at or closer than this.
OBSTACLE_STOP_CM = 25.0

# Resume only once the path opens beyond this. The gap between stop and
# clear is hysteresis, so the robot does not flap right at 25cm.
OBSTACLE_CLEAR_CM = 32.0

# Consecutive readings below OBSTACLE_STOP_CM before stopping,
# so one noisy echo does not halt the robot.
OBSTACLE_CONFIRM_READINGS = 2

# Path must stay clear this long before resuming.
OBSTACLE_CLEAR_SECONDS = 0.5

# Ignore distance readings older than this (serial hiccup safety).
ULTRASONIC_STALE_SECONDS = 1.0


# ----------------------------
# Camera configuration
# ----------------------------
CAMERA_INDEX = 0

# Capture resolution:
# 640x360 is a good balance:
# - not too heavy like 640x480
# - better than 320x240 for future QR/stream
# - 16:9 camera view is usually cleaner
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
CAMERA_FPS = 30

# Detection resolution:
# Line following does not need full 640.
# This keeps latency low.
DETECT_WIDTH = 320
DETECT_HEIGHT = 180

CAMERA_REOPEN_DELAY_SECONDS = 1.0
CAMERA_UNAVAILABLE_LOG_INTERVAL = 5.0
JPEG_QUALITY = 78


# ----------------------------
# Line detection configuration
# ----------------------------

# Use grayscale dark-line detection, faster and usually cleaner than HSV
# for this floor. The tape appears as dark gray in the camera, so this
# cannot be too low or the robot will miss the line.
BLACK_THRESHOLD = 135

# Also require local contrast: the line must be this much darker than the
# nearby floor/wall around it. Keep this gentle because the tape is broad
# enough that an aggressive contrast filter can erase the middle of it.
BLACK_LOCAL_CONTRAST = 10

# Always keep pixels this dark even if local contrast is weak.
BLACK_STRONG_THRESHOLD = 90

# Ignore tiny noise at 320x180 detection resolution.
MIN_LINE_AREA_BOTTOM = 90
MIN_LINE_AREA_LOOKAHEAD = 60

# Two ROI system:
# Bottom ROI = current position
# Lookahead ROI = upcoming curve
BOTTOM_ROI_TOP_RATIO = 0.68
BOTTOM_ROI_BOTTOM_RATIO = 0.98

LOOKAHEAD_ROI_TOP_RATIO = 0.42
LOOKAHEAD_ROI_BOTTOM_RATIO = 0.68

# Tolerance at DETECT_WIDTH=320.
# Wide center band prevents small angle changes from immediately pushing
# the detected line outside the forward zone.
CENTER_TOLERANCE = 72

# How much lookahead affects steering.
# Higher = reacts earlier to curves.
LOOKAHEAD_WEIGHT = 0.65
BOTTOM_WEIGHT = 1.00

# Crop edges if robot body/wheels appear.
CROP_LEFT_RATIO = 0.00
CROP_RIGHT_RATIO = 1.00

# Optional: if lighting is noisy, use Otsu threshold instead of fixed threshold.
# Fixed threshold is faster and more predictable.
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
        # Non-blocking read of firmware output.
        # We only care about the auto-pushed "DIST:<cm>" lines.
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
        # Returns (distance_cm, measured_at). (None, 0.0) when unknown.
        # distance_cm <= 0 means no echo (nothing in sensor range).
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


# ----------------------------
# Recovery state
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
obstacle_below_count = 0
obstacle_clear_since = None
obstacle_last_reading_at = 0.0


def set_recovery_phase(phase):
    global recovery_phase, recovery_phase_started_at

    recovery_phase = phase
    recovery_phase_started_at = time.time()
    print(f"Recovery phase: {phase}")


def set_servo_angle(angle):
    global servo_angle_now, servo_last_moved_at

    if servo_angle_now == angle:
        return

    # Rate-limit physical servo moves. Callers re-assert the angle every
    # frame, so a skipped move is retried once the interval has passed.
    now = time.time()
    if now - servo_last_moved_at < SERVO_MOVE_MIN_INTERVAL:
        return

    servo_angle_now = angle
    servo_last_moved_at = now
    robot.send_servo(angle, force=True)


def start_recovery():
    global recovery_mode, recovery_found_streak, last_follow_action

    recovery_mode = True
    recovery_found_streak = 0
    last_follow_action = None
    set_servo_angle(SERVO_RIGHT_SCAN_ANGLE)
    set_recovery_phase("TURN_RIGHT")


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
    global obstacle_mode, obstacle_clear_since, recovery_mode, last_follow_action

    obstacle_mode = True
    obstacle_clear_since = None
    last_follow_action = None

    # Abandon any recovery; it restarts fresh if the line is still lost
    # after the obstacle clears.
    if recovery_mode:
        recovery_mode = False
        set_recovery_phase("NORMAL")

    robot.send("S", force=True)
    set_servo_angle(SERVO_OBSTACLE_ANGLE)
    print("Obstacle detected: robot stopped, servo lifted.")


def exit_obstacle_mode():
    global obstacle_mode, obstacle_below_count, last_line_seen_at

    obstacle_mode = False
    obstacle_below_count = 0
    set_servo_angle(SERVO_DEFAULT_ANGLE)

    # Give the camera a fresh lost-line window so recovery does not fire
    # the instant the robot resumes.
    last_line_seen_at = time.time()
    print("Obstacle cleared: resuming line following.")


def update_obstacle_state():
    global obstacle_below_count, obstacle_clear_since, obstacle_last_reading_at

    distance, measured_at = robot.get_distance()

    # No fresh reading, or this reading was already processed:
    # keep the current state.
    if distance is None or measured_at <= obstacle_last_reading_at:
        return

    obstacle_last_reading_at = measured_at

    # distance <= 0 means no echo, i.e. nothing in sensor range.
    blocked = 0 < distance <= OBSTACLE_STOP_CM

    if not obstacle_mode:
        if blocked:
            obstacle_below_count += 1

            if obstacle_below_count >= OBSTACLE_CONFIRM_READINGS:
                enter_obstacle_mode()
        else:
            obstacle_below_count = 0

        return

    now = time.time()

    if distance <= 0 or distance > OBSTACLE_CLEAR_CM:
        if obstacle_clear_since is None:
            obstacle_clear_since = now
        elif now - obstacle_clear_since >= OBSTACLE_CLEAR_SECONDS:
            exit_obstacle_mode()
    else:
        obstacle_clear_since = None


# ----------------------------
# Vision helpers
# ----------------------------
def create_black_mask(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Small blur reduces noise but stays fast.
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
    # Resize only for detection.
    # The stream still shows captured resolution.
    frame_small = cv2.resize(
        frame,
        (DETECT_WIDTH, DETECT_HEIGHT),
        interpolation=cv2.INTER_AREA,
    )

    h, w = frame_small.shape[:2]

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

    # Steering reference. Shift with CENTER_OFFSET_PX if the camera is
    # mounted slightly off-center.
    reference_x = (w // 2) + CENTER_OFFSET_PX

    bottom_error = 0
    lookahead_error = 0

    if bottom["found"]:
        bottom_error = bottom["center_x"] - reference_x

    if lookahead["found"]:
        lookahead_error = lookahead["center_x"] - reference_x

    # Combine current position and upcoming curve.
    # Bottom is most important, lookahead helps early turn.
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

    # Scale detection coordinates to full stream frame for drawing.
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


# ----------------------------
# Decision logic
# ----------------------------
def choose_follow_command(detection):
    global smoothed_error, last_follow_action

    # Smooth the steering error so a single noisy frame does not flip
    # the command back and forth.
    smoothed_error = (
        ERROR_SMOOTHING_ALPHA * detection["weighted_error"]
        + (1.0 - ERROR_SMOOTHING_ALPHA) * smoothed_error
    )

    error = smoothed_error
    abs_error = abs(error)

    # Hysteresis: while already pivoting, keep pivoting until the line
    # is well inside the tolerance band instead of flapping at the edge.
    was_turning = last_follow_action is not None and last_follow_action[1] in ("Q", "E")

    if was_turning:
        forward_tolerance = CENTER_TOLERANCE * TURN_RELEASE_RATIO
    else:
        forward_tolerance = CENTER_TOLERANCE

    if abs_error <= forward_tolerance:
        decision = "FORWARD"
        command = "F"

        # Slow a little if lookahead says curve is coming.
        curve_pressure = abs(detection["lookahead_error"])
        if curve_pressure > CENTER_TOLERANCE * 2:
            speed = MIN_FOLLOW_SPEED
        else:
            speed = ROBOT_SPEED

        last_follow_action = (decision, command, speed)
        return last_follow_action

    # Dynamic turn speed based on how far line is from center.
    # Stronger error = stronger pivot.
    turn_speed = LINE_TURN_SPEED

    if abs_error > 90:
        turn_speed = LINE_TURN_SPEED + 4
    elif abs_error > 55:
        turn_speed = LINE_TURN_SPEED + 2

    turn_speed = max(18, min(24, turn_speed))

    if error < 0:
        last_follow_action = ("PIVOT LEFT", "Q", turn_speed)
    else:
        last_follow_action = ("PIVOT RIGHT", "E", turn_speed)

    return last_follow_action


def choose_recovery_command(detection):
    global last_line_seen_at, recovery_found_streak

    now = time.time()
    elapsed = now - recovery_phase_started_at

    # Found line during recovery.
    # Require a few consecutive frames before leaving recovery, so one
    # noisy frame does not flap the mode and the servo.
    if detection["found_line"]:
        last_line_seen_at = now
        recovery_found_streak += 1

        if recovery_found_streak >= RECOVERY_EXIT_FOUND_FRAMES:
            stop_recovery()
            return choose_follow_command(detection)

        return "RECOVERY CONFIRM LINE", "S", 0

    recovery_found_streak = 0

    if recovery_phase == "TURN_RIGHT":
        set_servo_angle(SERVO_RIGHT_SCAN_ANGLE)

        if elapsed < RECOVERY_TURN_SECONDS:
            return "RECOVERY TURN RIGHT", "E", RECOVERY_TURN_SPEED

        robot.send("S", force=True)
        set_recovery_phase("CHECK_RIGHT")
        return "RECOVERY CHECK RIGHT", "S", 0

    if recovery_phase == "CHECK_RIGHT":
        if elapsed < RECOVERY_CHECK_SECONDS:
            return "RECOVERY CHECK RIGHT", "S", 0

        set_servo_angle(SERVO_LEFT_SCAN_ANGLE)
        set_recovery_phase("TURN_LEFT")
        return "RECOVERY PREP LEFT", "S", 0

    if recovery_phase == "TURN_LEFT":
        set_servo_angle(SERVO_LEFT_SCAN_ANGLE)

        if elapsed < RECOVERY_TURN_SECONDS:
            return "RECOVERY TURN LEFT", "Q", RECOVERY_TURN_SPEED

        robot.send("S", force=True)
        set_recovery_phase("CHECK_LEFT")
        return "RECOVERY CHECK LEFT", "S", 0

    if recovery_phase == "CHECK_LEFT":
        if elapsed < RECOVERY_CHECK_SECONDS:
            return "RECOVERY CHECK LEFT", "S", 0

        set_servo_angle(SERVO_RIGHT_SCAN_ANGLE)
        set_recovery_phase("TURN_RIGHT")
        return "RECOVERY RESTART", "S", 0

    set_recovery_phase("TURN_RIGHT")
    return "RECOVERY RESET", "S", 0


def decide_robot_action(detection):
    global last_line_seen_at, last_follow_action

    now = time.time()

    if obstacle_mode:
        # Hold position with the camera lifted. Keep the line timer
        # fresh so recovery does not fire the moment the path clears.
        set_servo_angle(SERVO_OBSTACLE_ANGLE)
        last_line_seen_at = now
        return "OBSTACLE STOP", "S", 0

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

    # Briefly keep moving only if the last command was forward. Repeating
    # a pivot after the line disappears makes the robot rotate farther away
    # from the line and enter recovery with a bigger error.
    if last_follow_action is not None:
        decision, command, speed = last_follow_action

        if command == "F":
            return "LINE LOST COAST", command, speed

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

    # Draw center/tolerance lines based on stream size.
    # Uses the (possibly offset) steering reference, not the raw middle.
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
        f"MODE {mode_text} | PHASE {recovery_phase} | DIST {distance_text}",
        (10, 124),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 255, 255),
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
    # V4L2 often gives lower latency on Raspberry Pi/Linux.
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

    # Ask camera for MJPG to reduce USB bandwidth/CPU pressure.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    # Important: reduce old-frame buffering.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        cap.release()
        return None

    # Throw away a few startup frames.
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

            # Grab/retrieve helps keep latest frame fresher on some cameras.
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

            # Some cameras ignore requested resolution, so resize stream frame for consistency.
            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA)

            robot.poll_responses()
            update_obstacle_state()

            detection = detect_line(frame)
            decision, command, speed = decide_robot_action(detection)

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
        Capture 640x360, detect 320x180. Two ROI line tracking: lookahead + bottom.
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
