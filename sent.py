#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO1_PIN 4

Servo servo1;

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n=== ESP32 Servo Debug Ready ===");

  // attach with safe pulse range for SG90 (500..2400us)
  servo1.attach(SERVO1_PIN, 500, 2400);
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
  } else {
    Serial.println("Unknown command key.");
  }
}
