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

# Servo default angle.
# Python will send A90 to ESP32 when the program starts.
SERVO_DEFAULT_ANGLE = 10

# Start slow first. Increase later only after stable.
ROBOT_SPEED = 18
LINE_TURN_SPEED = 24
SEARCH_TURN_SPEED = 22

COMMAND_SEND_INTERVAL = 0.04
SAME_COMMAND_INTERVAL = 0.16

# Lost line behavior
LOST_LINE_GRACE_SECONDS = 0.12
SEARCH_DIRECTION_MEMORY_SECONDS = 1.0


# ----------------------------
# Camera / line configuration
# ----------------------------
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30

CAMERA_REOPEN_DELAY_SECONDS = 1.0
CAMERA_UNAVAILABLE_LOG_INTERVAL = 5.0
JPEG_QUALITY = 82

# Use lower part of image. Robot should follow the line close to itself.
ROI_TOP_RATIO = 0.70

# Black line HSV range.
# If black line is missed, increase V from 100 to 120.
LOWER_LINE_HSV = (0, 0, 0)
UPPER_LINE_HSV = (180, 255, 100)

# Ignore tiny black noise.
MIN_LINE_AREA = 350

# Error tolerance from image center.
# Smaller = reacts more.
# Bigger = goes forward more easily.
CENTER_TOLERANCE = 70

# Optional: crop left/right edges if camera sees wheels/body.
CROP_LEFT_RATIO = 0.00
CROP_RIGHT_RATIO = 1.00


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

        try:
            self.ser = serial.Serial(port, baud_rate, timeout=0.05)
            time.sleep(2)
            print(f"Connected to robot on {port}")

            # Set servo to default angle when Python starts.
            self.send("A", SERVO_DEFAULT_ANGLE, force=True)
            print(f"Servo default angle set to {SERVO_DEFAULT_ANGLE}")

        except serial.SerialException as exc:
            print(f"Serial not connected: {exc}")
            print("Video server will run, but robot commands will not be sent.")

    def send(self, command, speed=ROBOT_SPEED, force=False):
        if command == "S":
            payload = "S\n"
        else:
            payload = f"{command}{speed}\n"

        now = time.time()
        min_interval = SAME_COMMAND_INTERVAL if payload == self.last_payload else COMMAND_SEND_INTERVAL

        if not force and now - self.last_sent_at < min_interval:
            return

        if not self._write_payload(payload):
            return

        self.last_payload = payload
        self.last_sent_at = now
        print("Sent:", payload.strip())

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
# Line detection state
# ----------------------------
last_seen_line_at = 0.0
last_line_side = "LEFT"


def get_mask(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(
        hsv,
        np.array(LOWER_LINE_HSV, dtype=np.uint8),
        np.array(UPPER_LINE_HSV, dtype=np.uint8),
    )

    # Remove small noise and fill small gaps.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def detect_line(frame):
    global last_seen_line_at, last_line_side

    h, w = frame.shape[:2]

    roi_top = int(ROI_TOP_RATIO * h)
    crop_left = int(CROP_LEFT_RATIO * w)
    crop_right = int(CROP_RIGHT_RATIO * w)

    roi = frame[roi_top:h, crop_left:crop_right]

    mask = get_mask(roi)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    decision = "NO LINE"
    command = "S"
    speed = 0
    center_x = None
    center_y = None
    line_area = 0
    error = 0
    found_line = False

    if contours:
        largest = max(contours, key=cv2.contourArea)
        line_area = cv2.contourArea(largest)

        if line_area >= MIN_LINE_AREA:
            M = cv2.moments(largest)

            if M["m00"] > 0:
                local_center_x = int(M["m10"] / M["m00"])
                local_center_y = int(M["m01"] / M["m00"])

                center_x = crop_left + local_center_x
                center_y = roi_top + local_center_y

                error = center_x - (w // 2)
                found_line = True
                last_seen_line_at = time.time()

                if error < 0:
                    last_line_side = "LEFT"
                else:
                    last_line_side = "RIGHT"

                if abs(error) <= CENTER_TOLERANCE:
                    decision = "FORWARD"
                    command = "F"
                    speed = ROBOT_SPEED
                elif error < 0:
                    decision = "PIVOT LEFT"
                    command = "Q"
                    speed = LINE_TURN_SPEED
                else:
                    decision = "PIVOT RIGHT"
                    command = "E"
                    speed = LINE_TURN_SPEED

    if not found_line:
        time_since_seen = time.time() - last_seen_line_at

        # Very short grace period: stop instead of instantly spinning from one bad frame.
        if last_seen_line_at > 0 and time_since_seen <= LOST_LINE_GRACE_SECONDS:
            decision = "LINE LOST GRACE"
            command = "S"
            speed = 0

        # Search toward the last side where the line was seen.
        elif last_seen_line_at > 0 and time_since_seen <= SEARCH_DIRECTION_MEMORY_SECONDS:
            if last_line_side == "LEFT":
                decision = "SEARCH LEFT"
                command = "Q"
                speed = SEARCH_TURN_SPEED
            else:
                decision = "SEARCH RIGHT"
                command = "E"
                speed = SEARCH_TURN_SPEED

        # If lost for too long, stop for safety.
        else:
            decision = "NO LINE STOP"
            command = "S"
            speed = 0

    debug = {
        "roi_top": roi_top,
        "crop_left": crop_left,
        "crop_right": crop_right,
        "center_x": center_x,
        "center_y": center_y,
        "decision": decision,
        "command": command,
        "speed": speed,
        "line_area": line_area,
        "error": error,
        "mask": mask,
    }

    return debug


def draw_debug(frame, debug):
    h, w = frame.shape[:2]

    roi_top = debug["roi_top"]
    crop_left = debug["crop_left"]
    crop_right = debug["crop_right"]
    center_x = debug["center_x"]
    center_y = debug["center_y"]
    decision = debug["decision"]
    command = debug["command"]
    speed = debug["speed"]
    line_area = debug["line_area"]
    error = debug["error"]

    color = (0, 255, 0) if "NO LINE" not in decision and "LOST" not in decision else (0, 0, 255)

    # ROI rectangle
    cv2.rectangle(frame, (crop_left, roi_top), (crop_right, h), (0, 255, 0), 2)

    # Center and tolerance lines
    cv2.line(frame, (w // 2, roi_top), (w // 2, h), (255, 255, 0), 2)
    cv2.line(frame, (w // 2 - CENTER_TOLERANCE, roi_top), (w // 2 - CENTER_TOLERANCE, h), (120, 120, 0), 1)
    cv2.line(frame, (w // 2 + CENTER_TOLERANCE, roi_top), (w // 2 + CENTER_TOLERANCE, h), (120, 120, 0), 1)

    # Detected line center
    if center_x is not None and center_y is not None:
        cv2.circle(frame, (center_x, center_y), 9, (0, 0, 255), -1)
        cv2.line(frame, (w // 2, center_y), (center_x, center_y), (0, 0, 255), 2)

    cv2.putText(
        frame,
        f"{decision} | CMD {command} | SPD {speed}",
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        color,
        2,
    )

    cv2.putText(
        frame,
        f"AREA {int(line_area)} | ERR {int(error)} | SERVO {SERVO_DEFAULT_ANGLE}",
        (10, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 0),
        2,
    )


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
    frame = np.zeros((240, 520, 3), dtype=np.uint8)

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


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    if not cap.isOpened():
        cap.release()
        return None

    ok, frame = cap.read()

    if ok and frame is not None:
        print(f"Using OpenCV camera index {CAMERA_INDEX}")
        return cap

    print(f"OpenCV camera index {CAMERA_INDEX} opened but did not return frames.")
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

            ok, frame = cap.read()

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

            debug = detect_line(frame)

            robot.send(
                debug["command"],
                debug["speed"],
            )

            draw_debug(frame, debug)
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
            time.sleep(0.01)
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
            max-width: 640px;
            height: auto;
            border: 2px solid #333;
            background: black;
        }

        .hint {
            color: #bbb;
            font-size: 14px;
            margin: 10px auto 18px;
            max-width: 640px;
            padding: 0 14px;
        }
    </style>
</head>
<body>
    <h1>Robot Line Detection</h1>
    <div class="hint">
        Watch AREA and ERR. If line is visible but AREA is small/zero, adjust HSV V or lighting.
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