#include "SMA.hpp"
#include <Control_Surface.h>
#include <Wire.h>

using namespace CS;

USBMIDI_Interface midi;
PBValue pb {MCU::VOLUME_1};
PitchBendSender<10> sender;
Hysteresis<4, uint16_t, uint16_t> hyst;
bool prevTouched = false;

SMA<1, uint16_t, uint16_t> filt;

void setup() {
    Wire.begin();          // join i2c bus
    Wire.setClock(400000); // higher i2c speed
    Serial.begin(115200);  // start serial for output
    pinMode(LED_BUILTIN, OUTPUT);
    Control_Surface.begin();
}

const uint8_t slave_addr = 8;

void loop() {
    Control_Surface.loop();

    static Timer<millis> timer = 5;
    if (timer) {
        uint8_t maxlen = 1 + 4 * 2;
        Wire.requestFrom(slave_addr, maxlen);
        uint8_t buf[maxlen];
        uint8_t i = 0;
        while (Wire.available() && i < maxlen)
            buf[i++] = Wire.read();
        if (i > 0) {
            bool touched = buf[0] & (1 << 0);
            if (touched != prevTouched) {
                touched ? Control_Surface.sendNoteOn(MCU::FADER_TOUCH_1, 127)
                        : Control_Surface.sendNoteOff(MCU::FADER_TOUCH_1, 127);
                prevTouched = touched;
            }
        }
        if (i >= 3 && prevTouched) {
            uint16_t faderpos;
            memcpy(&faderpos, &buf[1], 2);
            if (hyst.update(faderpos)) {
                sender.send(hyst.getValue(), MCU::VOLUME_1);
            }
        }
        static uint16_t prevSetpoint = 0;
        uint16_t setpoint = filt(pb.getValue() >> 4);
        Serial.println(setpoint);
        if (setpoint != prevSetpoint) {
            digitalWrite(LED_BUILTIN, HIGH);
            uint16_t idx = 0;
            uint16_t data = setpoint;
            data |= idx << 12;
            Wire.beginTransmission(slave_addr);
            Wire.write(reinterpret_cast<const uint8_t *>(&data), 2);
            Wire.endTransmission();
            pb.clearDirty();
            digitalWrite(LED_BUILTIN, LOW);
            prevSetpoint = setpoint;
        }
    }
}
