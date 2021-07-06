#pragma once
#include "Registers.hpp"

extern const int16_t touch_sense_thres;
extern const uint16_t touch_sense_stickiness;
extern const int16_t touch_sense_discharge;

template <uint8_t Idx>
struct TouchSense {
    [[nodiscard]] static bool touch_pin_read();
    [[nodiscard]] static bool touch_pin_is_input();
    static void touch_pin_output();
    static void touch_pin_input();

    static void begin();
    static void update();

    volatile static bool touched;

    const static uint8_t touch_pin_bit;

    static int16_t touchcounter;
    static uint16_t stickytouched;
};

template <uint8_t Idx>
[[nodiscard]] inline bool TouchSense<Idx>::touch_pin_read() {
    return bit_is_set(PINB, touch_pin_bit);
}
template <uint8_t Idx>
[[nodiscard]] inline bool TouchSense<Idx>::touch_pin_is_input() {
    return bit_is_clear(DDRB, touch_pin_bit);
}
template <uint8_t Idx>
inline void TouchSense<Idx>::touch_pin_output() {
    sbi(DDRB, touch_pin_bit);
}
template <uint8_t Idx>
inline void TouchSense<Idx>::touch_pin_input() {
    cbi(DDRB, touch_pin_bit);
}

template <uint8_t Idx>
volatile bool TouchSense<Idx>::touched = false;

template <uint8_t Idx>
int16_t TouchSense<Idx>::touchcounter = 0;
template <uint8_t Idx>
uint16_t TouchSense<Idx>::stickytouched = 0;

template <>
const uint8_t TouchSense<0>::touch_pin_bit = 0;
template <>
const uint8_t TouchSense<1>::touch_pin_bit = 1;
template <>
const uint8_t TouchSense<2>::touch_pin_bit = 2;
template <>
const uint8_t TouchSense<3>::touch_pin_bit = 4;

template <uint8_t Idx>
void TouchSense<Idx>::begin() {
    cbi(PORTB, touch_pin_bit); // touch pin low
    sbi(DDRB, touch_pin_bit);  // touch pin output
}

// Capacitive touch logic
// ----------------------
//
// 0. pin mode is “output”, value is “low”
// 1. set the pin mode to “input”, touchcounter = 0
// 2. the pin will start charging through the external pull-up resistor
// 3. if the pin becomes “high”, record the touchcounter value
// 4. this value is proportional to the RC-time of the knob/pull-up resistor
//    circuit, since R is fixed, this can be used to infer C, the capacitance of
//    the knob: if touchcounter is higher than a threshold (i.e. if the
//    capacitance is higher), this probably means the knob is touched
// 5. set the pin mode to “output”, to start discharging the pin to 0V again
// 6. after touch_sense_discharge ticks, the pin has discharged, so switch to 
//    “input” mode and start charging again for the next RC-time measurement
//
// The “touched” status is sticky: it will remain set for at least 
// touch_sense_stickiness ticks. If the RC-time no longer exceeded the threshold
// during that period, the “touched” status is cleared.
//
// This could probably be optimized by combining the different touch sense 
// channels, but this is simpler and works fine.
template <uint8_t Idx>
void TouchSense<Idx>::update() {
    if (touch_pin_is_input()) { // touch pin is charging
        if (touch_pin_read()) { // touch pin is high
            if (touchcounter > touch_sense_thres) {
                touched = true;
                stickytouched = touch_sense_stickiness;
            }
            touch_pin_output(); // output mode, start discharging
            touchcounter = -touch_sense_discharge;
        }
    } else if (touchcounter == 0) {
        touch_pin_input(); // input mode, start charging
    }
    touchcounter++;
    if (stickytouched > 0) {
        stickytouched--;
        if (stickytouched == 0) {
            touched = false;
        }
    }
}