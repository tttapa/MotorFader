#pragma once
#include "Registers.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

inline void setupADC(uint8_t analog_input) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cbi(ADCSRA, ADEN); // Disable ADC

        cbi(ADMUX, REFS1); // Vcc reference
        sbi(ADMUX, REFS0); // Vcc reference

        cbi(ADMUX, ADLAR); // 8 least significant bits in ADCL

        sbi(ADCSRA, ADPS2); // Prescaler /128
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);

        ADMUX &= 0xF0;
        ADMUX |= analog_input; // Select the analog mux channel

        sbi(ADCSRB,
            ADTS2); // Trigger source = Timer/Counter0 Overflow (Table 23-6)
        cbi(ADCSRB, ADTS1);
        cbi(ADCSRB, ADTS0);

        cbi(ADCSRA, ADATE); // Auto trigger disable

        sbi(ADCSRA, ADIE); // ADC Interrupt Enable

        sbi(ADCSRA, ADEN); // Enable ADC
    }
}