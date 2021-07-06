#include <Arduino_Helpers.h>
#include <AH/PrintStream/PrintStream.hpp>
#include <AH/Hardware/FilteredAnalog.hpp>

#include <Wire.h>
FilteredAnalog<> analog = A0;

void setup() {
  Wire.begin();          // join i2c bus
  Wire.setClock(400000); // higher i2c speed
  Serial.begin(115200);  // start serial for output
  pinMode(LED_BUILTIN, OUTPUT);
  analog.setupADC();
}

const uint8_t slave_addr = 8;

void loop() {
    uint8_t maxlen = 1 + 4 * 2;
  Wire.requestFrom(slave_addr, maxlen);
  uint8_t buf[maxlen];
  uint8_t i = 0;
  while (Wire.available() && i < maxlen)
      buf[i++] = Wire.read();
  Serial << AH::HexDump(buf, i) << endl;
  if (i > 0)
    digitalWrite(LED_BUILTIN, buf[0] ? HIGH : LOW);

    if (analog.update()) {
        uint16_t idx = 0;
        uint16_t data = analog.getValue();
        data |= idx << 12;
        Wire.beginTransmission(slave_addr);
        Wire.write(reinterpret_cast<const uint8_t *>(&data), 2);
        Wire.endTransmission();
    }

  delay(5);
}
