#pragma once

#include <avr/io.h>
#include <util/delay.h> // F_CPU

#ifndef ARDUINO // Ensures that my IDE sees the correct frequency
#undef F_CPU
#define F_CPU 16000000UL
#endif

/// Timer 0 clock select (Table 14-9).
enum class Timer0Prescaler : uint8_t {
    None = 0b000,
    S1 = 0b001,
    S8 = 0b010,
    S64 = 0b011,
    S256 = 0b100,
    S1024 = 0b101,
    ExtFall = 0b110,
    ExtRise = 0b111,
    Invalid = 0xFF,
};

/// Timer 0 waveform generation mode (Table 14-8).
enum class Timer0WGMode : uint8_t {
    Normal = 0b000,
    PWM = 0b001,
    CTC = 0b010,
    FastPWM = 0b011,
    PWM_OCRA = 0b101,
    FastPWM_OCRA = 0b111,
};

// Convert the prescaler factor to the correct bit pattern to write to the
// TCCR0B register (Table 14-9).
constexpr inline Timer0Prescaler factorToTimer0Prescaler(uint16_t factor) {
    return factor == 1      ? Timer0Prescaler::S1
           : factor == 8    ? Timer0Prescaler::S8
           : factor == 64   ? Timer0Prescaler::S64
           : factor == 256  ? Timer0Prescaler::S256
           : factor == 1024 ? Timer0Prescaler::S1024
                            : Timer0Prescaler::Invalid;
}

#ifndef sbi
/// Set bit in register.
template <class R>
inline void sbi(R &reg, uint8_t bit) {
    reg |= (1u << bit);
}
#define sbi sbi
#endif
#ifndef cbi
/// Clear bit in register.
template <class R>
inline void cbi(R &reg, uint8_t bit) {
    reg &= ~(1u << bit);
}
#define cbi cbi
#endif
/// Write bit in register.
template <class R>
inline void wbi(R &reg, uint8_t bit, bool value) {
    value ? sbi(reg, bit) : cbi(reg, bit);
}

/// Set the clock source/prescaler of Timer0 (Table 14-9).
inline void setTimer0Prescaler(Timer0Prescaler ps) {
    if (ps == Timer0Prescaler::Invalid)
        return;
    wbi(TCCR0B, CS02, static_cast<uint8_t>(ps) & (1u << 2));
    wbi(TCCR0B, CS01, static_cast<uint8_t>(ps) & (1u << 1));
    wbi(TCCR0B, CS00, static_cast<uint8_t>(ps) & (1u << 0));
}

/// Set the wavefrom generation mode of Timer0 (Table 14-8).
inline void setTimer0WGMode(Timer0WGMode mode) {
    wbi(TCCR0B, WGM02, static_cast<uint8_t>(mode) & (1u << 2));
    wbi(TCCR0A, WGM01, static_cast<uint8_t>(mode) & (1u << 1));
    wbi(TCCR0A, WGM00, static_cast<uint8_t>(mode) & (1u << 0));
}