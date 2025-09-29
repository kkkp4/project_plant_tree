#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// --------- Pin Config ---------
#define SERVO1_PIN       4
#define SERVO2_PIN       5

// Stepper 1 (Step/Dir/EN)
#define STEP1_STEP_PIN   18
#define STEP1_DIR_PIN    19
#define STEP1_EN_PIN     23   // NEW: Enable Pin

// Stepper 2 (Step/Dir/EN)
#define STEP2_STEP_PIN   21
#define STEP2_DIR_PIN    22
#define STEP2_EN_PIN     27   // NEW: Enable Pin

// DC Motor1 (H-Bridge PWM + DIR)
#define MOTOR1_PWM_PIN   25
#define MOTOR1_DIR_PIN   26

// --------- Global Variables ---------
Servo servo1;
Servo servo2;

bool servo1State = false;     // Servo1 toggle 0° ↔ 150°
bool servo2State = false;     // Servo2 toggle 0° ↔ 90°
bool motor1State = false;     // DC Motor toggle ON/OFF

int speedVal = 100;           // ค่า speed จาก Pi
bool stopAll = false;         // STOP_ALL flag

// --------- Helper Functions ---------
void stepperMove(int stepPin, int dirPin, int enPin, bool forward, int steps, int spd) {
  if (stopAll) return;

  // เปิดการทำงานของ Stepper (Active Low)
  digitalWrite(enPin, LOW);
  digitalWrite(dirPin, forward ? HIGH : LOW);

  int delay_us = map(spd, 0, 255, 2000, 300); // speed control

  for (int i = 0; i < steps; i++) {
    if (stopAll) break;
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_us);
  }

  // ปิดการทำงานของ Stepper หลังจบการหมุน
  digitalWrite(enPin, HIGH);
}

void motor1Toggle(int spd) {
  if (motor1State) {
    ledcWrite(0, 0);
    Serial.println("Motor1 OFF");
    motor1State = false;
  } else {
    digitalWrite(MOTOR1_DIR_PIN, HIGH);
    int pwm = map(spd, 0, 255, 0, 255);
    ledcWrite(0, pwm);
    Serial.println("Motor1 ON");
    motor1State = true;
  }
}

void stopAllMotorsAndServos() {
  stopAll = true;

  // Servo reset
  servo1.write(0);
  servo2.write(0);

  // ปิด Stepper ทั้งหมด
  digitalWrite(STEP1_EN_PIN, HIGH);
  digitalWrite(STEP2_EN_PIN, HIGH);

  // DC Motor OFF
  ledcWrite(0, 0);
  motor1State = false;

  Serial.println("ALL STOP!");
}

// --------- Setup ---------
void setup() {
  Serial.begin(115200);

  // Servo setup
  servo1.attach(SERVO1_PIN);
  servo1.write(0);
  servo2.attach(SERVO2_PIN);
  servo2.write(0);

  // Stepper setup
  pinMode(STEP1_STEP_PIN, OUTPUT);
  pinMode(STEP1_DIR_PIN, OUTPUT);
  pinMode(STEP1_EN_PIN, OUTPUT);
  digitalWrite(STEP1_EN_PIN, HIGH); // เริ่มต้นปิด

  pinMode(STEP2_STEP_PIN, OUTPUT);
  pinMode(STEP2_DIR_PIN, OUTPUT);
  pinMode(STEP2_EN_PIN, OUTPUT);
  digitalWrite(STEP2_EN_PIN, HIGH); // เริ่มต้นปิด

  // Motor1 setup (PWM + DIR)
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  ledcSetup(0, 5000, 8); // channel 0, freq 5kHz, 8-bit
  ledcAttachPin(MOTOR1_PWM_PIN, 0);

  Serial.println("ESP32 ready (Servo1 + Servo2 + Stepper1 + Stepper2 + DC Motor1 + EN pins)!");
}

// --------- Loop ---------
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // แยก command และ speed
    int sep = cmd.indexOf(':');
    String action = cmd.substring(0, sep);
    int spd = cmd.substring(sep + 1).toInt();

    if (spd > 0) speedVal = spd;
    if (action != "STOP_ALL") stopAll = false;

    Serial.print("Recv: ");
    Serial.print(action);
    Serial.print(" | Speed: ");
    Serial.println(speedVal);

    // -------- Stepper control --------
    if (action == "STEP1_FWD") {
      stepperMove(STEP1_STEP_PIN, STEP1_DIR_PIN, STEP1_EN_PIN, true, 200, speedVal);
    }
    else if (action == "STEP1_BWD") {
      stepperMove(STEP1_STEP_PIN, STEP1_DIR_PIN, STEP1_EN_PIN, false, 200, speedVal);
    }
    else if (action == "STEP2_45") {
      stepperMove(STEP2_STEP_PIN, STEP2_DIR_PIN, STEP2_EN_PIN, true, 25, speedVal);
    }

    // -------- Servo1 control --------
    else if (action == "SERVO1_TOGGLE") {
      servo1State = !servo1State;
      servo1.write(servo1State ? 150 : 0);
    }

    // -------- Servo2 control --------
    else if (action == "SERVO2_TOGGLE") {
      servo2State = !servo2State;
      servo2.write(servo2State ? 90 : 0);
    }

    // -------- DC Motor1 control --------
    else if (action == "MOTOR1_TOGGLE") {
      motor1Toggle(speedVal);
    }

    // -------- STOP ALL --------
    else if (action == "STOP_ALL") {
      stopAllMotorsAndServos();
    }
  }
}
