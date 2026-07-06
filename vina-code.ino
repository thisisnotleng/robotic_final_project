#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <ESP32Servo.h>

Servo myServo;


// Motor pin configuration copied from test/sketch_may22b.ino.
// Vina's movement logic below is kept the same.
#define FL_PWM 14
#define FL_IN1 32
#define FL_IN2 27
#define FL_CH 8

#define FR_PWM 19
#define FR_IN1 23
#define FR_IN2 22
#define FR_CH 9

#define RL_PWM 33
#define RL_IN1 26
#define RL_IN2 25
#define RL_CH 10

#define RR_PWM 5
#define RR_IN1 21
#define RR_IN2 18
#define RR_CH 11

// Keep Vina's M1-M4 movement layout, but wire it to the old hardware map.
#define M1_PWM RL_PWM
#define M1_IN1 RL_IN1
#define M1_IN2 RL_IN2
#define M1_CH RL_CH

#define M2_PWM FL_PWM
#define M2_IN1 FL_IN1
#define M2_IN2 FL_IN2
#define M2_CH FL_CH

#define M3_PWM RR_PWM
#define M3_IN1 RR_IN1
#define M3_IN2 RR_IN2
#define M3_CH RR_CH

#define M4_PWM FR_PWM
#define M4_IN1 FR_IN1
#define M4_IN2 FR_IN2
#define M4_CH FR_CH

const uint16_t RECV_PIN = 36;   
const uint8_t LED_PIN = 12;      // LED pin

#define SERVO_PIN 17
#define TRIG_PIN 13
#define ECHO_PIN 39

  
long duration;


IRrecv irrecv(RECV_PIN);
decode_results results;
int angle = 50;

String cmd  = "";

void setup() {
  Serial.begin(115200);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  ledcSetup(M1_CH, 20000, 8);
  ledcAttachPin(M1_PWM, M1_CH);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  ledcSetup(M2_CH, 20000, 8);
  ledcAttachPin(M2_PWM, M2_CH);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  ledcSetup(M3_CH, 20000, 8);
  ledcAttachPin(M3_PWM, M3_CH);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  ledcSetup(M4_CH, 20000, 8);
  ledcAttachPin(M4_PWM, M4_CH);


  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  irrecv.enableIRIn(); 
  myServo.attach(SERVO_PIN);  
  myServo.write(angle);  
}

void move_backward(int speed){

  ledcWrite(M1_CH, speed);
  ledcWrite(M2_CH, speed);
  ledcWrite(M3_CH, speed);
  ledcWrite(M4_CH, speed);

  //M1
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);

  //M2
  digitalWrite(M2_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);

  //M3
  digitalWrite(M3_IN2, HIGH);
  digitalWrite(M3_IN1, LOW);

  //M4
  digitalWrite(M4_IN2, HIGH);
  digitalWrite(M4_IN1, LOW);
}

void move_forward(int speed){
  ledcWrite(M1_CH, speed);
  ledcWrite(M2_CH, speed);
  ledcWrite(M3_CH, speed);
  ledcWrite(M4_CH, speed);

  //M1
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);

  //M2
  digitalWrite(M2_IN2, HIGH);
  digitalWrite(M2_IN1, LOW);

  //M3
  digitalWrite(M3_IN2, LOW);
  digitalWrite(M3_IN1, HIGH);

  //M4
  digitalWrite(M4_IN2, LOW);
  digitalWrite(M4_IN1, HIGH);
}

void move_left(int rot_speed){
  ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, 0);
  ledcWrite(M3_CH, rot_speed);
  ledcWrite(M4_CH, rot_speed);
  //M1
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);

  //M2
  digitalWrite(M2_IN2, HIGH);
  digitalWrite(M2_IN1, LOW);

  //M3
  digitalWrite(M3_IN2, LOW);
  digitalWrite(M3_IN1, HIGH);

  //M4
  digitalWrite(M4_IN2, LOW);
  digitalWrite(M4_IN1, HIGH);
}

void move_right(int rot_speed){
  ledcWrite(M1_CH, rot_speed);
  ledcWrite(M2_CH, rot_speed);
  ledcWrite(M3_CH, 0);
  ledcWrite(M4_CH, 0);
  //M1
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);

  //M2
  digitalWrite(M2_IN2, HIGH);
  digitalWrite(M2_IN1, LOW);

  //M3
  digitalWrite(M3_IN2, HIGH);
  digitalWrite(M3_IN1, LOW);

  //M4
  digitalWrite(M4_IN2, HIGH);
  digitalWrite(M4_IN1, LOW);
}

void stop(){
  ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, 0);
  ledcWrite(M3_CH, 0);
  ledcWrite(M4_CH, 0);
}


float distance;
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

   if (duration == 0) {
    return -1;
  }

  return (duration * 0.0343) / 2.0;
}
  
  // if (duration == 0) {
  //   Serial.println("No echo");
  //   return 999; 
  // } else {
  //   distance = (duration * 0.0343) / 2.0;
  //   return (int)distance; 
  // }

int speed = 40;
int rot_speed = 60;

void loop() {
  
   
  float distance = readDistance();
  if (distance != -1) {
    Serial.print("Distance:");
    Serial.println(distance);
  }

 

  if (Serial.available()){

    cmd = Serial.readStringUntil('\n');
    cmd.trim();

    Serial.println(cmd);
    if (cmd == "Forward"){
      move_forward(speed);
    }
    else if (cmd == "Backward"){
      move_backward(speed);
    }
    else if (cmd == "Left"){
      move_left(rot_speed);
    }
    else if (cmd == "Right"){
      move_right(rot_speed);
    }
    else if (cmd == "Stop"){
      stop();
    }
  }
}
