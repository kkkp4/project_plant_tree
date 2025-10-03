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

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n=== ESP32 Servo Debug Ready ===");
  
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
  
  // attach with safe pulse range for SG90 (500..2400us)
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo1.writeMicroseconds(1500); // neutral
  servo1.attach(SERVO2_PIN, 500, 2400);
  servo1.writeMicroseconds(1500); // neutral
  delay(500);
  Serial.println("Servo attached on GPIO4 (500..2400us).");
}

String readLine() {
  // non-blocking readStringUntil is OK but we implement simple blocking read for debug
  String s = "";
  unsigned long start = millis();
  // wait up to 2000 ms for something
  while (millis() - start < 2000) {
    while (Serial.available()) {
      char c = Serial.read();
      s += c;
      if (c == '\n') return s;
    }
    delay(5);
  }
  return s;
}

void loop() {
  String cmd = readLine();
  if (cmd.length() == 0) {
    // no data received within timeout
    delay(10);
    return;
  }

  // trim CR/LF and spaces
  cmd.trim();
  //debugPrintRaw(cmd);

  // Simple parsing:
  int sep = cmd.indexOf(':');
  if (sep == -1) {
    Serial.println("No ':' found. Expected format: KEY:VALUE");
    return;
  }
  String key = cmd.substring(0, sep);
  String valStr = cmd.substring(sep + 1);
  valStr.trim();

  Serial.printf("Parsed KEY='%s', VAL='%s'\n", key.c_str(), valStr.c_str());

  if (key.equalsIgnoreCase("SERVO1")) {
    int angle = valStr.toInt(); // 0..180
    angle = constrain(angle, 0, 180);
    int us = map(angle, 0, 180, 500, 2400); // SG90 mapping
    servo1.writeMicroseconds(us);
    Serial.printf("-> Servo1 angle %d -> %d us\n", angle, us);
    }
    else if (key.equalsIgnoreCase("STEP1_FWD")) {
      int speedVal = valStr.toInt(); // 0..180
      stepperMove(STEP1_STEP_PIN, STEP1_DIR_PIN, STEP1_EN_PIN, true, 200, speedVal);
    }
    else if (key.equalsIgnoreCase("STEP1_BWD")) {
      int speedVal = valStr.toInt(); // 0..180
      stepperMove(STEP1_STEP_PIN, STEP1_DIR_PIN, STEP1_EN_PIN, false, 200, speedVal);
    }
    else if (key.equalsIgnoreCase("STEP2_45")) {
      int speedVal = valStr.toInt(); // 0..180
      stepperMove(STEP2_STEP_PIN, STEP2_DIR_PIN, STEP2_EN_PIN, true, 25, speedVal);
    } 
    else if (key.equalsIgnoreCase("SERVO2")) {
    int angle = valStr.toInt(); // 0..180
    angle = constrain(angle, 0, 180);
    int us = map(angle, 0, 180, 500, 2400); // SG90 mapping
    servo2.writeMicroseconds(us);
    Serial.printf("-> Servo1 angle %d -> %d us\n", angle, us);
    }
    else if (action == "MOTOR1_TOGGLE") {
      int speedVal = 110;
      motor1Toggle(speedVal);
    } else {
    Serial.println("Unknown command key.");
  }
}
