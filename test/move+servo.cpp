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
#define ENCODER_L_A 34
#define ENCODER_L_B 35
#define ENCODER_R_A 32
#define ENCODER_R_B 33

volatile long ticks_left = 0;
volatile long ticks_right = 0;

const float wheel_diameter_mm = 47.0;
const int gear_ratio = 90;
const int encoder_cpr = 11;
const int ticks_per_rev = encoder_cpr * gear_ratio;
const float wheel_circumference_mm = 3.1416 * wheel_diameter_mm;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

int motorSpeed = 150;
unsigned long lastReport = 0;

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

  // ===== MOTOR PWM SETUP =====
  for (int i = 0; i < 4; i++) {
    ledcSetup(motors[i].channel1, 20000, 8);
    ledcSetup(motors[i].channel2, 20000, 8);
    ledcAttachPin(motors[i].in1, motors[i].channel1);
    ledcAttachPin(motors[i].in2, motors[i].channel2);
  }

  // ===== ENCODER SETUP =====
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, CHANGE);

  // ===== SERVO SETUP =====
  cameraServo.setPeriodHertz(50);
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(180);

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

    // ===== SERVO COMMAND =====
    if (direction == 'V' && sep > 0) {
      int angle = cmd.substring(sep + 1).toInt();
      angle = constrain(angle, 0, 180);
      servoAngle = angle;
      cameraServo.write(servoAngle);

      Serial.println("SERVO: " + String(servoAngle));
      return;
    }

    // ===== MOTOR SPEED =====
    if (sep > 0) {
      motorSpeed = cmd.substring(sep + 1).toInt();
      motorSpeed = constrain(motorSpeed, 0, 255);
    }

    if (direction == 'F') {
      for (int i = 0; i < 4; i++)
        setMotor(i, 'F', motorSpeed);
    }
    else if (direction == 'B') {
      for (int i = 0; i < 4; i++)
        setMotor(i, 'B', motorSpeed);
    }
    else if (direction == 'L') {
      setMotor(0, 'B', motorSpeed);
      setMotor(1, 'B', motorSpeed);
      setMotor(2, 'F', motorSpeed);
      setMotor(3, 'F', motorSpeed);
    }
    else if (direction == 'R') {
      setMotor(0, 'F', motorSpeed);
      setMotor(1, 'F', motorSpeed);
      setMotor(2, 'B', motorSpeed);
      setMotor(3, 'B', motorSpeed);
    }
    else if (direction == 'S') {
      stopAll();
    }

    Serial.println("OK: " + cmd);
  }

  // ===== REPORT ENCODER DATA =====
  if (millis() - lastReport >= 500) {

    static long lastL = 0;
    static long lastR = 0;

    long nowL = ticks_left;
    long nowR = ticks_right;

    long deltaL = nowL - lastL;
    long deltaR = nowR - lastR;

    float vL = (deltaL * mm_per_tick) / 0.5;
    float vR = (deltaR * mm_per_tick) / 0.5;

    float dL = nowL * mm_per_tick;
    float dR = nowR * mm_per_tick;

    Serial.print("L_TICKS:"); Serial.print(nowL);
    Serial.print(",R_TICKS:"); Serial.print(nowR);
    Serial.print(",L_VEL:"); Serial.print(vL, 2);
    Serial.print(",R_VEL:"); Serial.print(vR, 2);
    Serial.print(",L_DIST:"); Serial.print(dL, 2);
    Serial.print(",R_DIST:"); Serial.println(dR, 2);

    lastL = nowL;
    lastR = nowR;
    lastReport = millis();
  }
}
