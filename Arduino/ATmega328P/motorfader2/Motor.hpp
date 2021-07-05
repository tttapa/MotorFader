#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

inline void setupMotorTimer(bool phase_correct_pwm, Timer0Prescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setTimer0WGMode(phase_correct_pwm ? Timer0WGMode::PWM
                                          : Timer0WGMode::FastPWM);
        setTimer0Prescaler(prescaler);
        sbi(TCCR0A, COM0B1); // Table 14-6, 14-7 Compare Output Mode
        sbi(DDRB, 4);        // GPIO output mode
        sbi(DDRD, 5);
    }
}

inline void motorForward(uint8_t speed) {
    // Fast PWM (Table 14-6):
    //   Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode).
    // Phase Correct PWM (Table 14-7):
    //   Clear OC2B on compare match when up-counting. Set OC2B on compare match
    //   when down-counting.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(TCCR0A, COM0B0);
        cbi(PORTD, 4);
        OCR0B = speed;
    }
}

inline void motorBackward(uint8_t speed) {
    // Fast PWM (Table 14-6):
    //   Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode).
    // Phase Correct PWM (Table 14-7):
    //   Set OC2B on compare match when up-counting. Clear OC2B on compare match
    //   when down-counting.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sbi(TCCR0A, COM0B0);
        sbi(PORTD, 4);
        OCR0B = speed;
    }
}
