#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// --------- Pin Config ---------
#define SERVO1_PIN       4

// Stepper 1 (ใช้ Step/Dir)
#define STEP1_STEP_PIN   18
#define STEP1_DIR_PIN    19

// Stepper 2 (ใช้ Step/Dir)
#define STEP2_STEP_PIN   21
#define STEP2_DIR_PIN    22

// DC Motor1 (ตัวอย่างใช้ H-Bridge แบบ PWM + DIR)
#define MOTOR1_PWM_PIN   25
#define MOTOR1_DIR_PIN   26

// --------- Global Variables ---------
Servo servo1;
bool servo1State = false;     // toggle 0° ↔ 150°
bool motor1State = false;     // toggle ทำงาน ↔ หยุด

int speedVal = 100;           // ค่า speed จาก Pi (ใช้คุม stepper และมอเตอร์)

bool stopAll = false;         // flag สำหรับ STOP_ALL

// --------- Helper Functions ---------
void stepperMove(int stepPin, int dirPin, bool forward, int steps, int spd) {
  if (stopAll) return;
  digitalWrite(dirPin, forward ? HIGH : LOW);

  int delay_us = map(spd, 0, 255, 2000, 300); // speed control

  for (int i = 0; i < steps; i++) {
    if (stopAll) break;
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_us);
  }
}

void motor1Toggle(int spd) {
  if (motor1State) {
    // OFF
    ledcWrite(0, 0);
    Serial.println("Motor1 OFF");
    motor1State = false;
  } else {
    // ON (forward)
    digitalWrite(MOTOR1_DIR_PIN, HIGH);
    int pwm = map(spd, 0, 255, 0, 255);
    ledcWrite(0, pwm);
    Serial.println("Motor1 ON");
    motor1State = true;
  }
}

void stopAllMotorsAndServos() {
  stopAll = true;

  // Servo reset to 0
  servo1.write(0);

  // Stepper จะหยุดทันที (loop เช็ค stopAll)
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

  // Stepper setup
  pinMode(STEP1_STEP_PIN, OUTPUT);
  pinMode(STEP1_DIR_PIN, OUTPUT);
  pinMode(STEP2_STEP_PIN, OUTPUT);
  pinMode(STEP2_DIR_PIN, OUTPUT);

  // Motor1 setup (PWM + DIR)
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);         // channel 0, freq 5kHz, 8-bit
  ledcAttachPin(MOTOR1_PWM_PIN, 0);

  Serial.println("ESP32 ready (Servo1 + Stepper1 + Stepper2 + DC Motor1)!");
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
      stepperMove(STEP1_STEP_PIN, STEP1_DIR_PIN, true, 200, speedVal);
    }
    else if (action == "STEP1_BWD") {
      stepperMove(STEP1_STEP_PIN, STEP1_DIR_PIN, false, 200, speedVal);
    }
    else if (action == "STEP2_45") {
      stepperMove(STEP2_STEP_PIN, STEP2_DIR_PIN, true, 25, speedVal);
    }

    // -------- Servo1 control --------
    else if (action == "SERVO1_TOGGLE") {
      servo1State = !servo1State;
      servo1.write(servo1State ? 150 : 0);
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
