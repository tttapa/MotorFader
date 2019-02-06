#include "Hysteresis.hpp"
#include "PID.hpp"
#include "Signal.h"

#define sbi(port,bit) \
  (port) |= (1 << (bit))
#define cbi(port,bit) \
  (port) &= ~(1 << (bit))

const float Ts = 1.0 / 781.25; // 781.25 Hz
const uint8_t interruptCounter = round(Ts * 8e6 / 256);

void setupADC() {
  cli();
  
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

  sei();
}

void setupPWM() {
  cli();
  // Fast PWM
  cbi(TCCR0B, WGM02); // 11-5 Waveform Generation Mode Bit Description
  sbi(TCCR0A, WGM01);
  sbi(TCCR0A, WGM00); 

  // Prescaler /1 → 8 MHz / 256 / 1 = 31.25 kHz
  cbi(TCCR0B, CS02); // 11-6 Clock Select Bit Description
  cbi(TCCR0B, CS01); 
  sbi(TCCR0B, CS00);

  sbi(DDRB, 1); // Output
  sei();
}

void setupMotor() {
  setupPWM();
  sbi(DDRB, 4);
}

void motorBackward(uint8_t speed) {
  // Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode)
  sbi(TCCR0A, COM0B1); // 11-3 Compare Output Mode, Fast PWM Mode
  sbi(TCCR0A, COM0B0);
  sbi(PORTB, 4);
  OCR0B = speed;
}

void motorForward(uint8_t speed) {
  // Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
  sbi(TCCR0A, COM0B1); // 11-3 Compare Output Mode, Fast PWM Mode
  cbi(TCCR0A, COM0B0);
  cbi(PORTB, 4);
  OCR0B = speed;
}
  
int main() {
  setupMotor();
  setupADC();
  sbi(TIMSK, TOIE0); // Enable timer 0 overflow interrupt
  sbi(DDRB, 0); // pin 0 output
  sbi(DDRB, 2); // pin 2 output
  while(true) {
    loop();
  }  
}

volatile int16_t adcval = -1;

const int16_t knee = 50;

uint8_t activation(int16_t val) {
  if (val == 0)
    return 0;
  else
    return map(val, 0, 255, knee, 255);
}

void updateController(int16_t adcval) {
  static EMA_f ema = 0.75;
  static Hysteresis<2> hyst;
  static uint8_t counter = 0;
  static size_t index = 0;
  static PID pid = {
    3,       // Kp
    8,       // Ki
    -3.2e-2, // Kd
    Ts,
  };

  if (counter++ >= 0) {
    counter = 0;
  }
  if (counter == 0) {
    uint16_t newSetPoint = pgm_read_byte(signal + index) * 4;
    pid.setpoint = newSetPoint;
    index++;
    if (index == len)
      index = 0;
  }
  
  hyst.update(ema(adcval));
  
  uint16_t position = hyst.getValue() << 2;
  int16_t control = pid.update(position);

#if 1
  if (control >= 0)
    motorForward(activation(control));
  else
    motorBackward(activation(-control));
#endif
}

void loop() {
  if (adcval >= 0) {
    sbi(PINB, 0); // toggle pin 0
    updateController(adcval);
    adcval = -1;
    sbi(PINB, 0); // toggle pin 0
  }
}

ISR(TIMER0_OVF_vect) {
  static uint8_t counter = 0;
  counter++;
  if (counter > interruptCounter) {
    counter = 0;
    sbi(ADCSRA, ADEN); // enable ADC
  }
}

ISR(ADC_vect) {
  cbi(ADCSRA, ADEN); // disable ADC
  uint8_t low = ADCL;
  adcval = ((uint16_t)ADCH << 8) | low;
}
