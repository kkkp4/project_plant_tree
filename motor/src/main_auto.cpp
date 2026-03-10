#include <Arduino.h>
#include <ESP32Servo.h>

// ================= MOTOR STRUCT =================
struct Motor {
  int in1;
  int in2;
  int channel1;
  int channel2;
};

Motor motors[4] = {
  {4, 18, 0, 1},
  {14, 27, 2, 3},
  {19, 23, 4, 5},
  {25, 26, 6, 7}
};

// ================= SERVO =================
#define SERVO_PIN 5
Servo cameraServo;
int servoAngle = 90;

// ================= ENCODER =================
#define ENCODER_L_A 21
#define ENCODER_L_B 22
#define ENCODER_R_A 16
#define ENCODER_R_B 17

volatile long ticks_left = 0;
volatile long ticks_right = 0;

const float wheel_diameter_mm = 47.0;
const int gear_ratio = 90;
const int encoder_cpr = 11;
const int ticks_per_rev = encoder_cpr * gear_ratio;

const float wheel_circumference_mm = 3.1416 * wheel_diameter_mm;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

int motorSpeed = 120;
unsigned long lastReport = 0;

// ===== distance move control =====
bool moveByDistance = false;
float targetDistance = 0;
float startDistance = 0;

// ===== backward distance control (เพิ่ม) =====
bool moveBackwardByDistance = false;
float targetDistanceBack = 0;
float startDistanceBack = 0;


// ================= ENCODER ISR =================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCODER_L_A) == digitalRead(ENCODER_L_B))
    ticks_left--;
  else
    ticks_left++;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCODER_R_A) == digitalRead(ENCODER_R_B))
    ticks_right--;
  else
    ticks_right++;
}


// ================= MOTOR CONTROL =================
void setMotor(int id, char dir, int speed) {

  if (id < 0 || id > 3) return;

  if (dir == 'F') {
    ledcWrite(motors[id].channel1, speed);
    ledcWrite(motors[id].channel2, 0);
  }
  else if (dir == 'B') {
    ledcWrite(motors[id].channel1, 0);
    ledcWrite(motors[id].channel2, speed);
  }
  else {
    ledcWrite(motors[id].channel1, 0);
    ledcWrite(motors[id].channel2, 0);
  }
}

void stopAll() {
  for (int i = 0; i < 4; i++)
    setMotor(i, 'S', 0);
}


// ================= SETUP =================
void setup() {

  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {

    ledcSetup(motors[i].channel1, 20000, 8);
    ledcSetup(motors[i].channel2, 20000, 8);

    ledcAttachPin(motors[i].in1, motors[i].channel1);
    ledcAttachPin(motors[i].in2, motors[i].channel2);
  }

  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, CHANGE);

  cameraServo.setPeriodHertz(50);
  cameraServo.attach(SERVO_PIN);

  Serial.println("ESP32 Robot Ready!");
}


// ================= LOOP =================
void loop() {

  // ===== RECEIVE COMMAND =====
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    char direction = cmd.charAt(0);
    int sep = cmd.indexOf(':');

    // ===== SERVO =====
    if (direction == 'V' && sep > 0) {

      int angle = cmd.substring(sep + 1).toInt();
      angle = constrain(angle, 0, 180);

      servoAngle = angle;
      cameraServo.write(servoAngle);

      Serial.println("SERVO: " + String(servoAngle));
      return;
    }

    // ===== MOVE FORWARD DISTANCE =====
    if (direction == 'D' && sep > 0) {

      ticks_left = 0;
      ticks_right = 0;

      targetDistance = cmd.substring(sep + 1).toFloat();

      float dL = (ticks_left * mm_per_tick) / 10.0;
      float dR = (-ticks_right * mm_per_tick) / 10.0;

      startDistance = (dL + dR) / 2.0;

      moveByDistance = true;
      cameraServo.write(servoAngle);
      for (int i = 0; i < 4; i++)
        setMotor(i, 'F', motorSpeed);

      Serial.println("MOVE_CM:" + String(targetDistance));
      return;
    }

    // ===== MOVE BACKWARD DISTANCE (เพิ่ม) =====
    if (cmd.startsWith("BD:")) {

      ticks_left = 0;
      ticks_right = 0;

      targetDistanceBack = cmd.substring(3).toFloat();

      float dL = (ticks_left * mm_per_tick) / 10.0;
      float dR = (-ticks_right * mm_per_tick) / 10.0;

      startDistanceBack = (dL + dR) / 2.0;

      moveBackwardByDistance = true;
      cameraServo.write(servoAngle);
      for (int i = 0; i < 4; i++)
        setMotor(i, 'B', motorSpeed);

      Serial.println("BACK_CM:" + String(targetDistanceBack));
      return;
    }

    if (sep > 0) {
      motorSpeed = cmd.substring(sep + 1).toInt();
      motorSpeed = constrain(motorSpeed, 0, 255);
    }

    // ===== NORMAL MOVE =====
    if (direction == 'F') {
      for (int i = 0; i < 4; i++)
        setMotor(i, 'F', motorSpeed);
      cameraServo.write(servoAngle);
    }

    else if (direction == 'B') {
      for (int i = 0; i < 4; i++)
        setMotor(i, 'B', motorSpeed);
      cameraServo.write(servoAngle);
    }

    else if (direction == 'L') {
      setMotor(0,'B',motorSpeed);
      setMotor(1,'B',motorSpeed);
      setMotor(2,'F',motorSpeed);
      setMotor(3,'F',motorSpeed);
      cameraServo.write(servoAngle);
    }

    else if (direction == 'R') {
      setMotor(0,'F',motorSpeed);
      setMotor(1,'F',motorSpeed);
      setMotor(2,'B',motorSpeed);
      setMotor(3,'B',motorSpeed);
      cameraServo.write(servoAngle);
    }

    if (direction == 'S') {
      stopAll();
      moveByDistance = false;
      moveBackwardByDistance = false;
      cameraServo.write(servoAngle);
    }

    Serial.println("OK: " + cmd);
  }


  // ===== FORWARD DISTANCE CHECK =====
  if (moveByDistance) {

    float dL = (ticks_left * mm_per_tick) / 10.0;
    float dR = (-ticks_right * mm_per_tick) / 10.0;

    float avg = (dL + dR) / 2.0;

    if (avg - startDistance >= targetDistance) {

      stopAll();
      moveByDistance = false;

      Serial.println("TARGET_REACHED");
    }
  }


  // ===== BACKWARD DISTANCE CHECK (เพิ่ม) =====
  if (moveBackwardByDistance) {

    float dL = (ticks_left * mm_per_tick) / 10.0;
    float dR = (-ticks_right * mm_per_tick) / 10.0;

    float avg = (dL + dR) / 2.0;

    if (abs(avg - startDistanceBack) >= targetDistanceBack) {

      stopAll();
      moveBackwardByDistance = false;

      Serial.println("BACK_TARGET_REACHED");
    }
  }


  // ===== REPORT =====
  if (millis() - lastReport >= 500) {

    float dL = (ticks_left * mm_per_tick) / 10.0;
    float dR = (-ticks_right * mm_per_tick) / 10.0;

    Serial.print("L_DIST_CM:");
    Serial.print(dL, 2);

    Serial.print(",R_DIST_CM:");
    Serial.println(dR, 2);

    lastReport = millis();
  }
}
