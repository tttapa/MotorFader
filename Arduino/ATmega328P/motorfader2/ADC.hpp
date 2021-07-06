#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

/// Enable the ADC with Vcc reference, /128 prescaler, auto trigger disabled,
/// ADC interrupt enabled.
inline void setupADC() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(ADCSRA, ADEN); // Disable ADC

        cbi(ADMUX, REFS1); // Vcc reference
        sbi(ADMUX, REFS0); // Vcc reference

        cbi(ADMUX, ADLAR); // 8 least significant bits in ADCL

        sbi(ADCSRA, ADPS2); // Prescaler /128
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);

        cbi(ADCSRA, ADATE); // Auto trigger disable
        sbi(ADCSRA, ADIE);  // ADC Interrupt Enable
        sbi(ADCSRA, ADEN);  // Enable ADC
    }
}