#include "Hysteresis.hpp"
#include "PID.hpp"

#define sbi(port,bit) \
  (port) |= (1 << (bit))
#define cbi(port,bit) \
  (port) &= ~(1 << (bit))

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

  // Prescaler /1
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

void wait() {
  for (size_t i = -1; i --> 0;) {
    asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
  }
}
  
int main() {
  setupMotor();
  setupADC();
  sbi(TIMSK, TOIE0); // Enable timer 0 overflow interrupt
  sbi(DDRB, 0); // pin 0 output
  while(true) {
    loop();
  }  
}

volatile int16_t adcval = -1;

const int16_t knee = 30 * 2;
// const int16_t knee = 40;

uint8_t activation(int16_t val) {
  if (val == 0)
    return 0;
  else
    return map(val, 0, 255, knee, 255);
}

void updateController(int16_t adcval) {
  static EMA_f ema = 0.8;
  static Hysteresis<2> hyst;
  static uint16_t counter = 0;
  static PID pid = {
    4,
   10,
   -3e-2,
   1e-3, // 1 kHz
  };
  pid.set(256 + (counter >> 1));
  hyst.update(ema(adcval));
  
  uint16_t output = hyst.getValue() << 2;
  int16_t control = pid.update(output);

#if 1
  if (control >= 0)
    motorForward(activation(control));
  else
    motorBackward(activation(-control));
#endif
  counter = (counter + 1) & 0x3FF;
    
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
  if (counter == 32) {
    counter = 0;
    sbi(ADCSRA, ADEN); // enable ADC
  }
}

ISR(ADC_vect) {
  cbi(ADCSRA, ADEN); // disable ADC
  uint8_t low = ADCL;
  adcval = ((uint16_t)ADCH << 8) | low;
}


