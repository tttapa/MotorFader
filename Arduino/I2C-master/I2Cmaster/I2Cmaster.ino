#include <Wire.h>

const uint8_t Reset               = 0x00;
const uint8_t RequestPosition     = 0x01;
const uint8_t SetSetpoint         = 0x02;
const uint8_t SetKp               = 0x03;
const uint8_t SetKi               = 0x04;
const uint8_t SetKd               = 0x05;
const uint8_t SetEMAp             = 0x06;
const uint8_t SetTouchSensitivity = 0x07;
const uint8_t Test                = 0x42;

const float kp = 3;
const float ki = 6;
const float kd = -3e-2;

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(250);
  Wire.beginTransmission(0x10);
  Wire.write(Reset);
  Wire.write(Reset);
  Wire.write(Reset);
  Wire.write(Reset);
  Wire.write(Reset); // Reset
  Wire.write(SetKp);
  Wire.write(reinterpret_cast<const uint8_t *>(&kp), sizeof(float));
  Wire.write(SetKi);
  Wire.write(reinterpret_cast<const uint8_t *>(&ki), sizeof(float));
  Wire.write(SetKd);
  Wire.write(reinterpret_cast<const uint8_t *>(&kd), sizeof(float));
  Wire.endTransmission();
  delay(250);
  // Wire.beginTransmission(0x10);
  // Wire.write(Test);
  // Wire.endTransmission();
}

void loop() {
  static uint8_t counter = 0;
  counter += 0x80;
  Wire.beginTransmission(0x10);
  Wire.write(SetSetpoint);
  Wire.write(counter + 0x3F);
  Wire.endTransmission();
  digitalWrite(LED_BUILTIN, counter > 0);
  delay(2000);
}
