#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

/// Configure Timer0 in either phase correct or fast PWM mode with the given
/// prescaler, enable output compare B.
inline void setupMotorTimer0(bool phase_correct, Timer0Prescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setTimer0WGMode(phase_correct ? Timer0WGMode::PWM
                                      : Timer0WGMode::FastPWM);
        setTimer0Prescaler(prescaler);
        sbi(TCCR0A, COM0B1); // Table 14-6, 14-7 Compare Output Mode
    }
}

/// Configure Timer2 in either phase correct or fast PWM mode with the given
/// prescaler, enable output compare B.
inline void setupMotorTimer2(bool phase_correct, Timer2Prescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setTimer2WGMode(phase_correct ? Timer2WGMode::PWM
                                      : Timer2WGMode::FastPWM);
        setTimer2Prescaler(prescaler);
        sbi(TCCR2A, COM2B1); // Table 14-6, 14-7 Compare Output Mode
    }
}

template <uint8_t Idx>
struct Motor {
    static void begin();
    static void forward(uint8_t speed);
    static void backward(uint8_t speed);
};

template <>
inline void Motor<0>::begin() {
    sbi(DDRD, 4);
    sbi(DDRD, 5);
}
template <>
inline void Motor<1>::begin() {
    sbi(DDRD, 7);
    sbi(DDRD, 6);
}
template <>
inline void Motor<2>::begin() {
    sbi(DDRD, 2);
    sbi(DDRD, 3);
}
template <>
inline void Motor<3>::begin() {
    // sbi(DDRB, 5);
    sbi(DDRB, 3);
}

// Fast PWM (Table 14-6):
//   Clear OC0B on Compare Match, set OC0B at BOTTOM (non-inverting mode).
// Phase Correct PWM (Table 14-7):
//   Clear OC0B on compare match when up-counting. Set OC0B on compare match
//   when down-counting.
template <>
inline void Motor<0>::forward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(TCCR0A, COM0B0);
        cbi(PORTD, 4);
        OCR0B = speed;
    }
}
template <>
inline void Motor<1>::forward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(TCCR0A, COM0A0);
        cbi(PORTD, 7);
        OCR0A = speed;
    }
}
template <>
inline void Motor<2>::forward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(TCCR2A, COM2B0);
        cbi(PORTD, 2);
        OCR2B = speed;
    }
}
template <>
inline void Motor<3>::forward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(TCCR2A, COM2B0);
        // cbi(PORTB, 5);
        OCR2A = speed;
    }
}

// Fast PWM (Table 14-6):
//   Set OC0B on Compare Match, clear OC0B at BOTTOM (inverting mode).
// Phase Correct PWM (Table 14-7):
//   Set OC0B on compare match when up-counting. Clear OC0B on compare match
//   when down-counting.
template <>
inline void Motor<0>::backward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sbi(TCCR0A, COM0B0);
        sbi(PORTD, 4);
        OCR0B = speed;
    }
}
template <>
inline void Motor<1>::backward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sbi(TCCR0A, COM0A0);
        sbi(PORTD, 7);
        OCR0A = speed;
    }
}
template <>
inline void Motor<2>::backward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sbi(TCCR2A, COM2B0);
        sbi(PORTD, 2);
        OCR2B = speed;
    }
}
template <>
inline void Motor<3>::backward(uint8_t speed) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sbi(TCCR2A, COM2B0);
        // sbi(PORTB, 5);
        OCR2A = speed;
    }
}