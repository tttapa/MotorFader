#include "Hysteresis.hpp"
#include "PID.hpp"
#include "Registers.hpp"
#include "Signal.h"
#include "util/delay.h"
#include <Arduino.h>

#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

constexpr uint8_t interruptCounter = 5;
constexpr unsigned prescaler_fac = 8;
constexpr auto prescaler = factorToTimer0Prescaler(prescaler_fac);
static_assert(prescaler != Timer0Prescaler::Invalid, "Invalid prescaler");
constexpr float Ts = 1. * prescaler_fac * (interruptCounter + 1) * 256 / F_CPU;

void setupADC() {
    cli();

    cbi(ADCSRA, ADEN);  // disable ADC
    sbi(ADCSRA, ADATE); // auto trigger enable

    cbi(ADMUX, REFS1); // Vcc reference
    sbi(ADMUX, REFS0); // Vcc reference

    cbi(ADMUX, ADLAR); // 8 least significant bits in ADCL

    sbi(ADCSRA, ADPS2); // prescaler /128
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS0);

    ADMUX &= 0xF0;
    ADMUX |= 0x00; // Select pin A0 as input to the analog mux

    // Trigger source = Timer/Counter0 Overflow (17-6)
    sbi(ADCSRB, ADTS2);
    cbi(ADCSRB, ADTS1);
    cbi(ADCSRB, ADTS0);

    sbi(ADCSRA, ADIE); // ADC Interrupt Enable

    sei();
}

void setupPWM() {
    cli();
    setTimer0WGMode(Timer0WGMode::FastPWM);
    setTimer0Prescaler(prescaler);

    sbi(DDRD, 5); // Output
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
    sbi(PORTD, 4);
    OCR0B = speed;
}

void motorForward(uint8_t speed) {
    // Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode)
    sbi(TCCR0A, COM0B1); // 11-3 Compare Output Mode, Fast PWM Mode
    cbi(TCCR0A, COM0B0);
    cbi(PORTD, 4);
    OCR0B = speed;
}

int main() {
    // init();
    // initVariant();
    Serial.begin(115200);
    setupMotor();
    setupADC();
    Serial.println(Ts * 1e6);
    sbi(DDRB, 5); // pin 13 output
    sbi(DDRD, 2); // pin 2 output
    // sbi(ADCSRA, ADEN); // enable ADC
    sbi(TIMSK0, TOIE0); // Enable timer 0 overflow interrupt
    while (true) {
        loop();
    }
}

volatile int16_t adcval = -1;
volatile bool touched = false;
volatile uint8_t touchval = 0xFF;

const int16_t knee = 50;

uint8_t activation(int16_t val) {
    if (val == 0)
        return 0;
    else
        return map(val, 0, 255, knee, 255);
}

void updateController(int16_t adcval) {
    static EMA_f ema = 0.85;
    static Hysteresis<2> hyst;
    static uint8_t counter = 0;
    static size_t index = 0;
    static PID pid = {
        3,  // Kp
        3,  // Ki
        -2.5e-2,  // Kd
        Ts, // Ts
    };

    if (counter++ >= 8) {
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
    Serial.print(pid.setpoint);
    Serial.print('\t');
    Serial.println(position);

#if 1
    if (touched)
        motorForward(0);
    else if (control >= 0)
        motorForward(activation(control));
    else
        motorBackward(activation(-control));
#endif
}

void loop() {
    noInterrupts();
    int16_t adcval_ = adcval;
    interrupts();
    if (adcval_ >= 0) {
        // Serial.println(adcval_);
        // sbi(PINB, 5); // toggle pin 13
        updateController(adcval_);
        noInterrupts();
        adcval = -1;
        interrupts();
        // sbi(PINB, 5); // toggle pin 13
    }
    // noInterrupts();
    // uint8_t touchval_ = touchval;
    // interrupts();
    // if (touchval_ != 0xFF) {
    //     noInterrupts();
    //     touchval = 0xFF;
    //     interrupts();
    //     Serial.println(touchval_);
    // }
}

constexpr int16_t touch_thres = 10;
constexpr uint8_t touch_stickiness = 255;

[[nodiscard]] inline bool touch_pin_read() { return (PIND & (1 << 2)) != 0; }
[[nodiscard]] inline bool touch_pin_is_input() {
    return (DDRD & (1 << 2)) == 0;
}
inline void touch_pin_output() { sbi(DDRD, 2); }
inline void touch_pin_input() { cbi(DDRD, 2); }

ISR(TIMER0_OVF_vect) {
    static uint8_t counter = 0;
    counter++;
    if (counter > interruptCounter) {
        counter = 0;
        sbi(ADCSRA, ADEN); // enable ADC
    }

    static int16_t touchcounter = 0;
    static uint8_t stickytouched = 0;
    if (touch_pin_is_input()) { // pin 2 is input
        if (touch_pin_read()) { // pin 2 is high
            if (touchcounter > touch_thres) {
                // sbi(PORTB, 5); // turn on pin 13
                touched = true;
                stickytouched = touch_stickiness;
            }
            touch_pin_output(); // output mode, start discharging
            touchval = touchcounter;
            touchcounter = -5;
        }
    } else if (touchcounter == 0) {
        touch_pin_input(); // input mode, start charging
    }
    touchcounter++;
    if (stickytouched > 0) {
        stickytouched--;
        if (stickytouched == 0) {
            // cbi(PORTB, 5);
            touched = false;
        }
    }
}

ISR(ADC_vect) {
    adcval = ADC;
    cbi(ADCSRA, ADEN); // disable ADC
    sbi(PINB, 5);      // toggle pin 13
}
