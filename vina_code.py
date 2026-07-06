from flask import Flask, Response
import cv2
import numpy as np
import serial
import time

app = Flask(__name__)

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2) 
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)



def get_mask(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower = (0, 0, 0)
    upper = (180, 255, 80)

    mask = cv2.inRange(hsv, lower, upper)

    return mask

last_distance = 999.0


def detect_color(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # red
    red1 = cv2.inRange(
        hsv,
        np.array([0,100,100]),
        np.array([10,255,255])
    )

    red2 = cv2.inRange(
        hsv,
        np.array([170,100,100]),
        np.array([180,255,255])
    )

    red = red1 + red2


    # green
    green = cv2.inRange(
        hsv,
        np.array([40,50,50]),
        np.array([80,255,255])
    )


    red_pixels = cv2.countNonZero(red)
    green_pixels = cv2.countNonZero(green)


    if red_pixels > 500:
        return "RED"

    elif green_pixels > 500:
        return "GREEN"

    return "NONE"

search_direction = "Left"
search_end_time = 0
last_distance = 999.0

def generate_frames():
    global last_distance, search_direction, search_end_time 
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        while ser.in_waiting:
            line = ser.readline().decode().strip()

            if line.startswith("Distance:"):
                try:
                    last_distance = float(line.split(":")[1])
                    print(f"Distance: {last_distance:.1f} cm")
                except:
                    pass

        h, w = frame.shape[:2]

        top = int(0.6 * h)
        roi = frame[top:h, :]

        mask = get_mask(roi)

        M = cv2.moments(mask)

        decision = "NO LINE"
        
        if last_distance < 30:

            decision = "Obstacle"

            command = "Stop"


            color = detect_color(frame)

            print("COLOR:", color)


            if color == "RED":

                decision = "RED - AVOID"

                # Stop
                ser.write(b"Stop\n")
                time.sleep(0.3)

                # Turn right
                ser.write(b"Right\n")
                time.sleep(0.8)

                # Move beside object
                ser.write(b"Forward\n")
                time.sleep(0.8)

                # Turn left
                ser.write(b"Left\n")
                time.sleep(0.8)

                # Move past object
                ser.write(b"Forward\n")
                time.sleep(0.8)

                # Turn left
                ser.write(b"Left\n")
                time.sleep(0.8)

                # Move back toward the line
                ser.write(b"Forward\n")
                time.sleep(1)

                # Face original direction
                ser.write(b"Right\n")
                time.sleep(0.8)

                continue

            elif color == "GREEN":

                decision = "GREEN - AVOID"

                # Stop
                ser.write(b"Stop\n")
                time.sleep(0.3)

                # Turn left
                ser.write(b"Left\n")
                time.sleep(0.8)

                # Move beside object
                ser.write(b"Forward\n")
                time.sleep(0.5)

                # Turn right
                ser.write(b"Right\n")
                time.sleep(0.5)

                # Move past object
                ser.write(b"Forward\n")
                time.sleep(0.8)

                # Turn left
                ser.write(b"Right\n")
                time.sleep(0.8)

                # Move back toward the line
                ser.write(b"Forward\n")
                time.sleep(1)

                # Face original direction
                ser.write(b"Left\n")
                time.sleep(0.8)

                continue
            else:

                decision = "Obstacle"

                current_time = time.time()

                if search_end_time == 0:
                    search_end_time = current_time + 2

                if current_time < search_end_time:
                    command = search_direction
                else:
                    if search_direction == "Left":
                        search_direction = "Right"
                    else:
                        search_direction = "Left"

                    search_end_time = current_time + 2
                    command = search_direction

        else:

            if M["m00"] > 0:

                # Line detected
                cx = int(M["m10"] / M["m00"])

                cv2.circle(roi, (cx, 50), 8, (0, 0, 255), -1)

                search_end_time = 0
                search_direction = "Left"

                if cx < w // 3:
                    decision = "LEFT"
                    command = "Left"

                elif cx > 2 * w // 3:
                    decision = "RIGHT"
                    command = "Right"

                else:
                    decision = "FORWARD"
                    command = "Forward"

            else:
                # No line detected -> search
                decision = "SEARCH LINE"

                current_time = time.time()

                if search_end_time == 0:
                    search_end_time = current_time + 4

                    # Stop briefly before searching
                    ser.write(b"Stop\n")
                    time.sleep(0.2)

                if current_time < search_end_time:
                    command = search_direction

                else:
                    if search_direction == "Left":
                        search_direction = "Right"
                    else:
                        search_direction = "Left"

                    search_end_time = current_time + 2

                    # Stop before changing direction
                    ser.write(b"Stop\n")
                    time.sleep(0.2)

                    command = search_direction


           

        # if last_command != command:
        ser.write((command+'\n').encode())
        # last_command = command
        # print(f"Sent command: {command}")
        print(f"Distance: {last_distance:.1f} cm | Sent: {command}")

        cv2.rectangle(frame, (0, top), (w, h), (0, 255, 0), 2)

        cv2.line(frame, (w//3, top), (w//3, h), (0, 0, 255), 2)
        cv2.line(frame, (2*w//3, top), (2*w//3, h), (255, 0, 0), 2)
        cv2.putText(frame, decision, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        # =========================
        # 6. Send to web
        # =========================
        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
