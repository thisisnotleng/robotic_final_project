#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <ESP32Servo.h>

Servo myServo;


int channel0 = 4;
int channel1 = 5;
int channel2 = 6;
int channel3 = 7;

const uint16_t RECV_PIN = 36;   
const uint8_t LED_PIN = 12;      // LED pin

#define TRIG_PIN 13
#define ECHO_PIN 39

  
long duration;


IRrecv irrecv(RECV_PIN);
decode_results results;
int angle = 50;

String cmd  = "";

void setup() {
  Serial.begin(115200);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  ledcSetup(channel0, 20000, 8);
  ledcAttachPin(33, channel0);

  pinMode(32, OUTPUT);
  pinMode(27, OUTPUT);
  ledcSetup(channel1, 20000, 8);
  ledcAttachPin(14, channel1);

  pinMode(21, OUTPUT);
  pinMode(18, OUTPUT);
  ledcSetup(channel2, 20000, 8);
  ledcAttachPin(05, channel2);

  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  ledcSetup(channel3, 20000, 8);
  ledcAttachPin(19, channel3);


  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  irrecv.enableIRIn(); 
  myServo.attach(17);  
  myServo.write(angle);  
}

void move_backward(int speed){

  ledcWrite(channel0, speed);
  ledcWrite(channel1, speed);
  ledcWrite(channel2, speed);
  ledcWrite(channel3, speed);

  //M1
  digitalWrite(26, HIGH);
  digitalWrite(25, LOW);

  //M2
  digitalWrite(27, LOW);
  digitalWrite(32, HIGH);

  //M3
  digitalWrite(18, HIGH);
  digitalWrite(21, LOW);

  //M4
  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
}

void move_forward(int speed){
  ledcWrite(channel0, speed);
  ledcWrite(channel1, speed);
  ledcWrite(channel2, speed);
  ledcWrite(channel3, speed);

  //M1
  digitalWrite(26, LOW);
  digitalWrite(25, HIGH);

  //M2
  digitalWrite(27, HIGH);
  digitalWrite(32, LOW);

  //M3
  digitalWrite(18, LOW);
  digitalWrite(21, HIGH);

  //M4
  digitalWrite(22, LOW);
  digitalWrite(23, HIGH);
}

void move_left(int rot_speed){
  ledcWrite(channel0, 0);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, rot_speed);
  ledcWrite(channel3, rot_speed);
  //M1
  digitalWrite(26, HIGH);
  digitalWrite(25, LOW);

  //M2
  digitalWrite(27, HIGH);
  digitalWrite(32, LOW);

  //M3
  digitalWrite(18, LOW);
  digitalWrite(21, HIGH);

  //M4
  digitalWrite(22, LOW);
  digitalWrite(23, HIGH);
}

void move_right(int rot_speed){
  ledcWrite(channel0, rot_speed);
  ledcWrite(channel1, rot_speed);
  ledcWrite(channel2, 0);
  ledcWrite(channel3, 0);
  //M1
  digitalWrite(26, LOW);
  digitalWrite(25, HIGH);

  //M2
  digitalWrite(27, HIGH);
  digitalWrite(32, LOW);

  //M3
  digitalWrite(18, HIGH);
  digitalWrite(21, LOW);

  //M4
  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
}

void stop(){
  ledcWrite(channel0, 0);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, 0);
  ledcWrite(channel3, 0);
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
