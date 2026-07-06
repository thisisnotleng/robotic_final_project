#include <Arduino.h>
#include <ESP32Servo.h>

#define DEFAULT_SPEED 25
#define COMMAND_TIMEOUT_MS 1200
#define SERIAL_DEBUG false

// -------------------- Smooth movement tuning --------------------

#define CONTROL_INTERVAL_MS 20

// How fast motor target changes.
// Smaller = smoother but slower response.
// Bigger = faster but more jerky. 
#define RAMP_STEP_UP 4
#define RAMP_STEP_DOWN 8

// Keep serial speeds literal so Python controls motor power directly.
#define MIN_ACTIVE_SPEED 1
#define MIN_ACTIVE_PWM 1

// Python already chooses safe line-following speeds.
#define MAX_DRIVE_SPEED 100

// Python also chooses pivot/search speeds.
#define MAX_PIVOT_SPEED 100

// Smooth curve turning.
// Inner wheels still move forward, but slower.
#define TURN_INNER_PERCENT 45

// Pivot turning.
// One side forward, one side backward.
// This makes rotation smoother by slightly reducing power.
#define PIVOT_PERCENT 100

// -------------------- Servo and ultrasonic pins --------------------

// Matches the basic working servo test sketch.
#define SERVO_PIN 17

// Your earlier ultrasonic setup.
#define ULTRASONIC_TRIG 13
#define ULTRASONIC_ECHO 39

// Automatic distance reporting.
// One short sample per interval keeps loop() responsive, and the
// Python side receives "DIST:<cm>" lines without having to ask.
#define ULTRASONIC_INTERVAL_MS 60

// Echo timeout ~1m max range. A short timeout keeps each sample fast
// (max ~6ms) so motor ramping stays smooth.
#define ULTRASONIC_TIMEOUT_US 6000

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Camera looks down at the line by default.
// Python lifts it to 10 when the ultrasonic sees an obstacle.
#define SERVO_DEFAULT_ANGLE 0

Servo cameraServo;

// -------------------- Motor pins --------------------

// Front Left motor
#define FL_PWM 14
#define FL_IN1 32
#define FL_IN2 27
#define FL_CH 8
#define FL_FORWARD true

// Front Right motor
#define FR_PWM 19
#define FR_IN1 23
#define FR_IN2 22
#define FR_CH 9
#define FR_FORWARD true

// Rear Left motor
#define RL_PWM 33
#define RL_IN1 26
#define RL_IN2 25
#define RL_CH 10
#define RL_FORWARD false

// Rear Right motor
#define RR_PWM 5
#define RR_IN1 21
#define RR_IN2 18
#define RR_CH 11
#define RR_FORWARD true

struct Motor {
  const char* name;
  int pwm;
  int in1;
  int in2;
  int channel;
  bool forwardDirection;
};

Motor motors[] = {
  {"Front Left",  FL_PWM, FL_IN1, FL_IN2, FL_CH, FL_FORWARD},
  {"Front Right", FR_PWM, FR_IN1, FR_IN2, FR_CH, FR_FORWARD},
  {"Rear Left",   RL_PWM, RL_IN1, RL_IN2, RL_CH, RL_FORWARD},
  {"Rear Right",  RR_PWM, RR_IN1, RR_IN2, RR_CH, RR_FORWARD}
};

const int motorCount = sizeof(motors) / sizeof(motors[0]);

// Motor speed range:
// positive = forward
// negative = backward
// zero = stop
int targetSpeed[4] = {0, 0, 0, 0};
int currentMotorSpeed[4] = {0, 0, 0, 0};

int currentSpeed = DEFAULT_SPEED;
unsigned long lastCommandAt = 0;
unsigned long lastControlUpdate = 0;

// Rolling ultrasonic samples for a simple median filter.
unsigned long lastUltrasonicAt = 0;
float distanceSamples[3] = {-1, -1, -1};
int distanceSampleIndex = 0;
float lastMedianDistanceCm = -1;

// -------------------- Function declarations --------------------

void setupMotorPins();
void setupServoAndUltrasonic();

void handleSerialCommands();
void handleCommand(String command);

void setTargetMotor(int index, int signedSpeed);
void setTargetAll(int fl, int fr, int rl, int rr);

void updateSmoothMotors();
int rampToward(int current, int target);
void applyMotorSpeed(int index, int signedSpeed);
void stopMotorImmediate(int index);

void forward(int speed);
void backward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void pivotLeft(int speed);
void pivotRight(int speed);

void smoothStop();
void emergencyStop();

int cleanSpeed(int speed, int maxSpeed);
int scaledSpeed(int speed, int percent);
int speedToPwm(int speed);

void setServoAngle(int angle);
void updateUltrasonic();
float medianDistanceCm();
float readUltrasonicSampleCm();

void printDebug(const char* message);

// -------------------- Setup and loop --------------------

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  setupMotorPins();
  setupServoAndUltrasonic();

  emergencyStop();

  Serial.println("Smooth serial motor controller ready");
  Serial.println("Commands: F30 B30 L30 R30 Q30 E30 S X A90 U");
}

void loop() {
  handleSerialCommands();

  if (lastCommandAt > 0 && millis() - lastCommandAt > COMMAND_TIMEOUT_MS) {
    smoothStop();
  }

  updateSmoothMotors();
  updateUltrasonic();
}

// -------------------- Setup helpers --------------------

void setupMotorPins() {
  for (int i = 0; i < motorCount; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);

    ledcSetup(motors[i].channel, 20000, 8);
    ledcAttachPin(motors[i].pwm, motors[i].channel);
  }
}

void setupServoAndUltrasonic() {
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  cameraServo.setPeriodHertz(50);
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(SERVO_DEFAULT_ANGLE);
}

// -------------------- Serial command handling --------------------

void handleSerialCommands() {
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command);
    }
  }
}

void handleCommand(String command) {
  char action = toupper(command.charAt(0));
  int value = currentSpeed;

  if (command.length() > 1) {
    String valueText = command.substring(1);

    if (valueText.charAt(0) == ':' || valueText.charAt(0) == ',') {
      valueText = valueText.substring(1);
    }

    valueText.trim();

    if (valueText.length() > 0) {
      value = valueText.toInt();
    }
  }

  lastCommandAt = millis();

  if (action == 'F') {
    currentSpeed = constrain(value, 0, 100);
    forward(currentSpeed);
  } else if (action == 'B') {
    currentSpeed = constrain(value, 0, 100);
    backward(currentSpeed);
  } else if (action == 'L') {
    currentSpeed = constrain(value, 0, 100);
    turnLeft(currentSpeed);
  } else if (action == 'R') {
    currentSpeed = constrain(value, 0, 100);
    turnRight(currentSpeed);
  } else if (action == 'Q') {
    currentSpeed = constrain(value, 0, 100);
    pivotLeft(currentSpeed);
  } else if (action == 'E') {
    currentSpeed = constrain(value, 0, 100);
    pivotRight(currentSpeed);
  } else if (action == 'S') {
    emergencyStop();
  } else if (action == 'X') {
    emergencyStop();
  } else if (action == 'V') {
    currentSpeed = constrain(value, 0, 100);
    Serial.print("Speed set to ");
    Serial.println(currentSpeed);
  } else if (action == 'A') {
    setServoAngle(value);
  } else if (action == 'U') {
    // Report the latest filtered distance without blocking.
    Serial.print("DIST:");
    Serial.println(lastMedianDistanceCm);
  } else {
    smoothStop();
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

// -------------------- Movement logic --------------------

void forward(int speed) {
  speed = cleanSpeed(speed, MAX_DRIVE_SPEED);

  setTargetAll(
    speed,
    speed,
    speed,
    speed
  );

  printDebug("Forward");
}

void backward(int speed) {
  speed = cleanSpeed(speed, MAX_DRIVE_SPEED);

  setTargetAll(
    -speed,
    -speed,
    -speed,
    -speed
  );

  printDebug("Backward");
}

void turnLeft(int speed) {
  speed = cleanSpeed(speed, MAX_DRIVE_SPEED);
  int innerSpeed = scaledSpeed(speed, TURN_INNER_PERCENT);

  // Left side slower, right side faster.
  // This is a smooth curve, not a spin.
  setTargetAll(
    innerSpeed,
    speed,
    innerSpeed,
    speed
  );

  printDebug("Smooth left");
}

void turnRight(int speed) {
  speed = cleanSpeed(speed, MAX_DRIVE_SPEED);
  int innerSpeed = scaledSpeed(speed, TURN_INNER_PERCENT);

  // Right side slower, left side faster.
  setTargetAll(
    speed,
    innerSpeed,
    speed,
    innerSpeed
  );

  printDebug("Smooth right");
}

void pivotLeft(int speed) {
  speed = cleanSpeed(speed, MAX_PIVOT_SPEED);
  int pivotSpeed = scaledSpeed(speed, PIVOT_PERCENT);

  // Left side backward, right side forward.
  // Soft pivot for finding lost line.
  setTargetAll(
    -pivotSpeed,
    pivotSpeed,
    -pivotSpeed,
    pivotSpeed
  );

  printDebug("Soft pivot left");
}

void pivotRight(int speed) {
  speed = cleanSpeed(speed, MAX_PIVOT_SPEED);
  int pivotSpeed = scaledSpeed(speed, PIVOT_PERCENT);

  // Left side forward, right side backward.
  setTargetAll(
    pivotSpeed,
    -pivotSpeed,
    pivotSpeed,
    -pivotSpeed
  );

  printDebug("Soft pivot right");
}

void smoothStop() {
  setTargetAll(0, 0, 0, 0);
  printDebug("Smooth stop");
}

void emergencyStop() {
  for (int i = 0; i < motorCount; i++) {
    targetSpeed[i] = 0;
    currentMotorSpeed[i] = 0;
    stopMotorImmediate(i);
  }

  printDebug("Emergency stop");
}

// -------------------- Smooth motor update --------------------

void setTargetMotor(int index, int signedSpeed) {
  targetSpeed[index] = constrain(signedSpeed, -100, 100);
}

void setTargetAll(int fl, int fr, int rl, int rr) {
  setTargetMotor(0, fl);
  setTargetMotor(1, fr);
  setTargetMotor(2, rl);
  setTargetMotor(3, rr);
}

void updateSmoothMotors() {
  if (millis() - lastControlUpdate < CONTROL_INTERVAL_MS) {
    return;
  }

  lastControlUpdate = millis();

  for (int i = 0; i < motorCount; i++) {
    currentMotorSpeed[i] = rampToward(currentMotorSpeed[i], targetSpeed[i]);
    applyMotorSpeed(i, currentMotorSpeed[i]);
  }
}

int rampToward(int current, int target) {
  if (current == target) {
    return current;
  }

  int step;

  // If slowing down or changing direction, decelerate faster.
  if (abs(target) < abs(current) || (current > 0 && target < 0) || (current < 0 && target > 0)) {
    step = RAMP_STEP_DOWN;
  } else {
    step = RAMP_STEP_UP;
  }

  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else {
    current -= step;
    if (current < target) current = target;
  }

  // When changing direction, pass through zero first.
  if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
    if (abs(current) < step) {
      current = 0;
    }
  }

  return current;
}

void applyMotorSpeed(int index, int signedSpeed) {
  Motor motor = motors[index];

  signedSpeed = constrain(signedSpeed, -100, 100);

  if (signedSpeed == 0) {
    stopMotorImmediate(index);
    return;
  }

  bool forwardDirection = signedSpeed > 0;
  int speed = abs(signedSpeed);
  int pwmValue = speedToPwm(speed);

  bool useForwardPins = forwardDirection == motor.forwardDirection;

  if (useForwardPins) {
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
  } else {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
  }

  ledcWrite(motor.channel, pwmValue);
}

void stopMotorImmediate(int index) {
  Motor motor = motors[index];

  ledcWrite(motor.channel, 0);
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
}

// -------------------- Speed helpers --------------------

int cleanSpeed(int speed, int maxSpeed) {
  speed = constrain(speed, 0, maxSpeed);

  if (speed > 0 && speed < MIN_ACTIVE_SPEED) {
    speed = MIN_ACTIVE_SPEED;
  }

  return speed;
}

int scaledSpeed(int speed, int percent) {
  int result = (speed * percent) / 100;

  if (result > 0 && result < MIN_ACTIVE_SPEED) {
    result = MIN_ACTIVE_SPEED;
  }

  return constrain(result, 0, 100);
}

int speedToPwm(int speed) {
  speed = constrain(speed, 0, 100);

  if (speed == 0) {
    return 0;
  }

  return map(speed, 0, 100, 0, 255);
}

// -------------------- Servo --------------------

void setServoAngle(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  cameraServo.write(angle);

  Serial.print("SERVO:");
  Serial.println(angle);
}

// -------------------- Ultrasonic --------------------

// Take one fast sample per interval and keep a median of the last 3.
// This spreads the sensor cost over time so motor control stays smooth,
// and pushes "DIST:<cm>" to the Python side automatically.
void updateUltrasonic() {
  if (millis() - lastUltrasonicAt < ULTRASONIC_INTERVAL_MS) {
    return;
  }

  lastUltrasonicAt = millis();

  distanceSamples[distanceSampleIndex] = readUltrasonicSampleCm();
  distanceSampleIndex = (distanceSampleIndex + 1) % 3;

  lastMedianDistanceCm = medianDistanceCm();

  Serial.print("DIST:");
  Serial.println(lastMedianDistanceCm);
}

float medianDistanceCm() {
  float valid[3];
  int validCount = 0;

  for (int i = 0; i < 3; i++) {
    if (distanceSamples[i] > 0) {
      valid[validCount] = distanceSamples[i];
      validCount++;
    }
  }

  if (validCount == 0) {
    return -1;
  }

  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (valid[j] < valid[i]) {
        float temp = valid[i];
        valid[i] = valid[j];
        valid[j] = temp;
      }
    }
  }

  return valid[validCount / 2];
}

float readUltrasonicSampleCm() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);

  if (duration == 0) {
    return -1;
  }

  float distanceCm = duration * 0.0343 / 2.0;
  return distanceCm;
}

// -------------------- Debug --------------------

void printDebug(const char* message) {
  if (SERIAL_DEBUG) {
    Serial.println(message);
  }
}
