#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 4   // Pin ที่ต่อกับ SG90

Servo myservo;
bool servoState = false;  // false = 70°, true = 150°

void setup() {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);  // SG90 ใช้ค่า default ก็พอ

  myservo.write(70);  // เริ่มที่ 70°
  Serial.println("=== SG90 Toggle Test ===");
  Serial.println("พิมพ์ z ใน Serial Monitor เพื่อหมุน Servo 70° ↔ 150°");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'z' || c == 'Z') {
      servoState = !servoState;  // toggle state

      if (servoState) {
        myservo.write(150);
        Serial.println("Servo → 150°");
      } else {
        myservo.write(70);
        Serial.println("Servo → 70°");
      }
    }
  }
}
