#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

/// Enable the ADC with Vcc reference, with the given prescaler, auto trigger 
/// disabled, ADC interrupt enabled.
inline void setupADC(ADCPrescaler prescaler) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(ADCSRA, ADEN); // Disable ADC

        cbi(ADMUX, REFS1); // Vcc reference
        sbi(ADMUX, REFS0); // Vcc reference

        cbi(ADMUX, ADLAR); // 8 least significant bits in ADCL

        setADCPrescaler(prescaler);

        cbi(ADCSRA, ADATE); // Auto trigger disable
        sbi(ADCSRA, ADIE);  // ADC Interrupt Enable
        sbi(ADCSRA, ADEN);  // Enable ADC
    }
}