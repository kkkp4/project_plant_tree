#include <Arduino.h>

struct Motor {
  int in1;
  int in2;
  int channel1;
  int channel2;
};

Motor motors[4] = {
  {4, 18, 0, 1},    // Motor 1
  {21, 22, 2, 3},   // Motor 2
  {19, 23, 4, 5},   // Motor 3 
  {32, 33, 6, 7}    // Motor 4
};

int motorSpeed = 150; // 0-255

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    ledcSetup(motors[i].channel1, 20000, 8); // 20kHz, 8-bit
    ledcSetup(motors[i].channel2, 20000, 8);
    ledcAttachPin(motors[i].in1, motors[i].channel1);
    ledcAttachPin(motors[i].in2, motors[i].channel2);
  }

  Serial.println("ESP32 + MDD3A Ready!");
}

void setMotor(int id, char dir, int speed) {
  if (id < 0 || id > 3) return;

  if (dir == 'F') {
    ledcWrite(motors[id].channel1, speed);
    ledcWrite(motors[id].channel2, 0);
  } else if (dir == 'B') {
    ledcWrite(motors[id].channel1, 0);
    ledcWrite(motors[id].channel2, speed);
  } else { // Stop
    ledcWrite(motors[id].channel1, 0);
    ledcWrite(motors[id].channel2, 0);
  }
}

void stopAll() {
  for (int i = 0; i < 4; i++) setMotor(i, 'S', 0);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    char direction = cmd.charAt(0);
    int sep = cmd.indexOf(':');
    if (sep > 0) {
      motorSpeed = cmd.substring(sep + 1).toInt();
      motorSpeed = constrain(motorSpeed, 0, 255);
    }

    if (direction == 'F') {
      for (int i = 0; i < 4; i++) setMotor(i, 'F', motorSpeed);
    } else if (direction == 'B') {
      for (int i = 0; i < 4; i++) setMotor(i, 'B', motorSpeed);
    } else if (direction == 'L') {
      // ซ้าย: มอเตอร์ซ้ายถอย, มอเตอร์ขวาเดินหน้า
      setMotor(0, 'B', motorSpeed);
      setMotor(1, 'B', motorSpeed);
      setMotor(2, 'F', motorSpeed);
      setMotor(3, 'F', motorSpeed);
    } else if (direction == 'R') {
      // ขวา
      setMotor(0, 'F', motorSpeed);
      setMotor(1, 'F', motorSpeed);
      setMotor(2, 'B', motorSpeed);
      setMotor(3, 'B', motorSpeed);
    } else if (direction == 'S') {
      stopAll();
    }

    Serial.println("OK: " + cmd);
  }
}
