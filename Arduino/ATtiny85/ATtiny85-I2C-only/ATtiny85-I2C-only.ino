extern "C" {
#include "twi_driver.h"
}

#define sbi(port,bit) \
  (port) |= (1 << (bit))
#define cbi(port,bit) \
  (port) &= ~(1 << (bit))

// ------------------------------------------------------------------------- //

int main() {
  setup();
  while (true) {
    loop();
  }
}

// ------------------------------------------------------------------------- //

const uint8_t TWI_slaveAddress = 0x10;

void setup() {
  USI_TWI_Slave_Initialise(TWI_slaveAddress);
  sbi(DDRB, 1); // pin 1 output
  sei();
}

// ------------------------------------------------------------------------- //

bool enabled = false;

void loop() {
  if (enabled) 
    sbi(PORTB, 1);
  else
    cbi(PORTB, 1);

  if (USI_TWI_Data_In_Receive_Buffer())
    parseByte(USI_TWI_Receive_Byte());
}

// ------------------------------------------------------------------------- //

const uint8_t Reset               = 0x00;
const uint8_t RequestPosition     = 0x01;
const uint8_t SetSetpoint         = 0x02;
const uint8_t SetKp               = 0x03;
const uint8_t SetKi               = 0x04;
const uint8_t SetKd               = 0x05;
const uint8_t SetEMAp             = 0x06;
const uint8_t SetTouchSensitivity = 0x07;
// TODO: set address

enum State : uint8_t {
  Initial = 0,
  ReceiveSetpoint,
  ReceiveFloat,
};

void parseByte(uint8_t data) {
  static uint8_t receivedFloat[sizeof(float)];
  static uint8_t floatByteIndex = 0;
  static uint8_t floatToStore;
  static State state = Initial;

  if (state == Initial) {
    if (data == Reset) {
      enabled = false;
    } else if (data == RequestPosition) {
      // if (touched)
      //   USI_TWI_Transmit_Byte(hyst.getValue());
    } else if (data == SetSetpoint) {
      state = ReceiveSetpoint;
    } else {
      state = ReceiveFloat;
      floatToStore = data;
      floatByteIndex = 0;
    }
  } else if (state == ReceiveSetpoint) {
    // pid.setpoint = data;
    enabled = true;
    state = Initial;
  } else if (state == ReceiveFloat) {
    enabled = false;
    receivedFloat[floatByteIndex++] = data;
    if (floatByteIndex == sizeof(float)) {
      setFloat(floatToStore, receivedFloat);
      state = Initial;
    }
  }
}

void setFloat(uint8_t floatToStore, const uint8_t *receivedFloat) {
  float f = *reinterpret_cast<const float *>(receivedFloat);
  switch (floatToStore) {
    /* case SetKp: pid.k_p = f; break;
    case SetKi: pid.k_i = f; break;
    case SetKd: pid.k_d = f; break;
    case SetEMAp: ema = f; break; */
    // TODO
    // case SetTouchSensitivity: ... break;
    default: ;
  }
}
