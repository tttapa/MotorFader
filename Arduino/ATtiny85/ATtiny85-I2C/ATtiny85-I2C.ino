#include "Hysteresis.hpp"
#include "PID.hpp"
extern "C" {
#include "twi_driver.h"
}

#define sbi(port,bit) \
  (port) |= (1 << (bit))
#define cbi(port,bit) \
  (port) &= ~(1 << (bit))

const float Ts = 1.0 / 781.25; // 781.25 Hz
const uint8_t interruptCounter = round(Ts * 8e6 / 256);

// ------------------------------- SETUP ADC ------------------------------- //
// ------------------------------------------------------------------------- //

void setupADC() {
  cbi(ADCSRA, ADEN); // disable ADC
  sbi(ADCSRA, ADATE); // auto trigger enable

  cbi(ADMUX, REFS1); // Vcc reference
  cbi(ADMUX, REFS0); // Vcc reference

  cbi(ADMUX, ADLAR); // 8 least significant bits in ADCL

  sbi(ADCSRA, ADPS2); // Prescaler /16
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  ADMUX &= 0xF0;
  ADMUX |= 0x03; // Select pin 3 as input to the analog mux

  // Trigger source = Timer/Counter0 Overflow (17-6)
  sbi(ADCSRB, ADTS2);
  cbi(ADCSRB, ADTS1);
  cbi(ADCSRB, ADTS0);

  sbi(ADCSRA, ADIE); // ADC Interrupt Enable
}

// ------------------------------- SETUP PWM ------------------------------- //
// ------------------------------------------------------------------------- //

void setupPWM() {
  // Fast PWM
  cbi(TCCR0B, WGM02); // 11-5 Waveform Generation Mode Bit Description
  sbi(TCCR0A, WGM01);
  sbi(TCCR0A, WGM00);

  // Prescaler /1 → 8 MHz / 256 / 1 = 31.25 kHz
  cbi(TCCR0B, CS02); // 11-6 Clock Select Bit Description
  cbi(TCCR0B, CS01);
  sbi(TCCR0B, CS00);

  sbi(DDRB, 1); // Output
}

void setupMotor() {
  setupPWM();
  sbi(DDRB, 4);
}

// ------------------------------------------------------------------------- //

const int16_t knee = 50;

uint8_t activation(int16_t val) {
  return val == 0 ? 0 : map(val, 0, 255, knee, 255);
}

void motorForward(uint8_t speed) {
  // Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
  sbi(TCCR0A, COM0B1); // 11-3 Compare Output Mode, Fast PWM Mode
  cbi(TCCR0A, COM0B0);
  cbi(PORTB, 4);
  OCR0B = speed;
}

void motorBackward(uint8_t speed) {
  // Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode)
  sbi(TCCR0A, COM0B1); // 11-3 Compare Output Mode, Fast PWM Mode
  sbi(TCCR0A, COM0B0);
  sbi(PORTB, 4);
  OCR0B = speed;
}

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
  cli();
  USI_TWI_Slave_Initialise(TWI_slaveAddress);
  setupMotor();
  setupADC();
  sbi(TIMSK, TOIE0); // Enable timer 0 overflow interrupt
  sei();
}

// ------------------------------------------------------------------------- //

volatile int16_t adcval = -1;
volatile bool touched = false;

bool enabled = false;
bool test = false;

void loop() {
  int16_t tmp_adcval = adcval;
  if (tmp_adcval >= 0) {
    updateController(tmp_adcval);
    adcval = -1;
  }

  if (USI_TWI_Data_In_Receive_Buffer())
    parseByte(USI_TWI_Receive_Byte());
}

// ------------------------------------------------------------------------- //

EMA_f ema = 0.85; // Filter applied to raw analog input
Hysteresis<2> hyst; // Hysteresis applied to filtered analog input
PID pid = {
  3,       // Kp
  8,       // Ki
  -3.2e-2, // Kd
  Ts,
};

void updateController(int16_t adcval) {
  hyst.update(ema(adcval));
  uint16_t position = hyst.getValue() << 2;
  int16_t control = pid.update(position);

  if (test) {
    motorForward(hyst.getValue());
    return;
  }

#if 1
  if (touched || !enabled)
    motorForward(0);
  else if (control >= 0)
    motorForward(activation(control));
  else
    motorBackward(activation(-control));
#endif
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
const uint8_t Test                = 0x42;
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
      test = false;
    } else if (data == RequestPosition) {
      if (touched)
        USI_TWI_Transmit_Byte(hyst.getValue());
    } else if (data == SetSetpoint) {
      state = ReceiveSetpoint;
    } else if (data == Test) {
      enabled = false;
      test = true;
    } else {
      state = ReceiveFloat;
      floatToStore = data;
      floatByteIndex = 0;
    }
  } else if (state == ReceiveSetpoint) {
    pid.setpoint = data << 2;
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
    case SetKp: pid.k_p = f; break;
    case SetKi: pid.k_i = f; break;
    case SetKd: pid.k_d = f; break;
    case SetEMAp: ema = f; break;
    // TODO
    // case SetTouchSensitivity: ... break;
    default: ;
  }
}

// ------------------------------------------------------------------------- //

ISR(TIMER0_OVF_vect) {
  static uint8_t counter = 0;
  counter++;
  if (counter > interruptCounter) {
    counter = 0;
    sbi(ADCSRA, ADEN); // enable ADC
  }

#if 0
  static uint8_t touchcounter = 0;
  if ((DDRB & _BV(2)) == 0) { // pin 2 is input
    if ((PINB & _BV(2)) != 0) { // pin 2 is high
      touched = touchcounter > 4;
      if (touchcounter > 4)
        sbi(PORTB, 0); // turn on pin 0
      else
        cbi(PORTB, 0);
      // cbi(PORTB, 2); // turn off internal pull-up
      sbi(DDRB, 2); // output mode, start discharging
    }
    touchcounter++;
  } else { // pin 2 is output
    cbi(DDRB, 2); // input mode
    touchcounter = 0;
    // sbi(PORTB, 2); // turn on internal pull-up
  }
#endif
}

// ------------------------------------------------------------------------- //

ISR(ADC_vect) {
  cbi(ADCSRA, ADEN); // disable ADC
  uint8_t low = ADCL;
  adcval = ((uint16_t)ADCH << 8) | low;
}
