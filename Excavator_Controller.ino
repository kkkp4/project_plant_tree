#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// Stepper1 (แขนหลัก)
#define STEP1_PIN1 25
#define STEP1_PIN2 26
#define STEP1_PIN3 27
#define STEP1_PIN4 14
AccelStepper stepper1(AccelStepper::FULL4WIRE, STEP1_PIN1, STEP1_PIN3, STEP1_PIN2, STEP1_PIN4);

// Stepper2 (แขนยก)
#define STEP2_PIN1 12
#define STEP2_PIN2 13
#define STEP2_PIN3 15
#define STEP2_PIN4 2
AccelStepper stepper2(AccelStepper::FULL4WIRE, STEP2_PIN1, STEP2_PIN3, STEP2_PIN2, STEP2_PIN4);

// Servo (bucket)
Servo myServo;
int servoPin = 5;
bool servoState = false;

void setup() {
  Serial.begin(115200);

  // init stepper pins low
  int stepPins[] = {STEP1_PIN1, STEP1_PIN2, STEP1_PIN3, STEP1_PIN4,
                    STEP2_PIN1, STEP2_PIN2, STEP2_PIN3, STEP2_PIN4};
  for (int i = 0; i < 8; i++) {
    pinMode(stepPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
  }

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  myServo.attach(servoPin);
  myServo.write(0);

  Serial.println("ESP32 Excavator Controller Ready!");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int sep = cmd.indexOf(':');
    String action = (sep > 0) ? cmd.substring(0, sep) : cmd;

    if (action == "STEP1_FWD") {
      stepper1.move(200); // ~1 rev
      while (stepper1.distanceToGo() != 0) stepper1.run();
    } else if (action == "STEP1_BWD") {
      stepper1.move(-200);
      while (stepper1.distanceToGo() != 0) stepper1.run();
    } else if (action == "STEP2_45") {
      int steps = 200 / 8; // ~45 deg
      stepper2.move(steps);
      while (stepper2.distanceToGo() != 0) stepper2.run();
    } else if (action == "SERVO_TOGGLE") {
      servoState = !servoState;
      myServo.write(servoState ? 150 : 0);
    }

    Serial.println("OK: " + cmd);
  }
}
