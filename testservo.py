#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 4

Servo myservo;

void setup() {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);

  delay(1000); // รอให้ Servo ready
  Serial.println("Start test...");

  myservo.write(70);
  Serial.println("Servo at 70°");
  delay(2000);

  myservo.write(150);
  Serial.println("Servo at 150°");
  delay(2000);

  myservo.write(70);
  Serial.println("Servo back to 70°");
}

void loop() {
  // ว่างไว้
}
