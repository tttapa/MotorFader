// Configuration and initialization of the analog-to-digital converter:
#include "ADC.hpp"
// PID controller:
#include "Controller.hpp"
// Configuration of PWM and Timer2/0 for driving the motor:
#include "Motor.hpp"
// Reference signal for testing the performance of the controller:
#include "Reference.hpp"
// Helpers for low-level AVR Timer2/0 and ADC registers
#include "Registers.hpp"

#include <Arduino.h>
#include <Arduino_Helpers.h>
#include <Wire.h>

#include <AH/Filters/EMA.hpp>

// ------------------------------ Description ------------------------------- //

// This sketch drives up to four motorized fader using a PID controller. It
// follows a reference trajectory defined in `Reference.hpp`. The motor is
// disabled when the user touches the knob of the fader.
//
// Everything is driven by Timer2, which runs (by default) at a rate of
// 31.250 kHz. This high rate is used to eliminate audible tones from the PWM
// drive for the motor. Timer0 is used for the PWM outputs of faders 3 and 4.
// Every 30 periods of Timer2 (960 µs), each analog input is sampled, and
// this causes the PID control loop to run in the main loop function.
// Capacitive sensing is implemented by measuring the RC time on the touch pin
// in the Timer2 interrupt handler. The “touched” status is sticky for >20 ms
// to prevent interference from the 50 Hz mains.

// -------------------------------- Hardware -------------------------------- //

// Fader 0:
// - A0:  wiper of the potentiometer          (ADMUX0)
// - D8:  touch pin of the knob               (PB0)
// - D2:  input 1A of L293D dual H-bridge 1   (PD2)
// - D3:  input 2A of L293D dual H-bridge 1   (OC2B)
//
// Fader 1:
// - A1:  wiper of the potentiometer          (ADMUX1)
// - D9:  touch pin of the knob               (PB1)
// - D13: input 3A of L293D dual H-bridge 1   (PB5)
// - D11: input 4A of L293D dual H-bridge 1   (OC2A)
//
// Fader 2:
// - A2:  wiper of the potentiometer          (ADMUX2)
// - D10: touch pin of the knob               (PB2)
// - D4:  input 1A of L293D dual H-bridge 2   (PD4)
// - D5:  input 2A of L293D dual H-bridge 2   (OC0B)
//
// Fader 3:
// - A3:  wiper of the potentiometer          (ADMUX3)
// - D12: touch pin of the knob               (PB4)
// - D7:  input 3A of L293D dual H-bridge 2   (PD7)
// - D6:  input 4A of L293D dual H-bridge 2   (OC0A)
//
// If fader 1 is unused:
// - D13: sLED or scope as overrun indicator  (PB5)
//
// Connect the outer connections of the potentiometers to ground and Vcc, it's
// recommended to add a 100 nF capacitor between each wiper and ground.
// Connect the 1,2EN and 3,4EN enable pins of the L293D chips to Vcc.
// Connect a 500kΩ pull-up resistor between each touch pin and Vcc.
// On an Arduino Nano, you can set an option to use pins A6/A7 instead of A2/A3.
// Note that D13 is often pulsed by the bootloader, which might cause the fader
// to move when resetting the Arduino. You can either disable this behavior in
// the bootloader, or use a different pin.

// ----------------------------- Configuration ------------------------------ //

// Print the control loop and interrupt frequencies to Serial at startup:
constexpr bool print_frequencies = true;

// Print the setpoint, actual position and control signal to Serial.
// Note that this slows down the control loop significantly, it goes from
// 29% to >83% CPU usage.
constexpr bool print_controller_signals = false;
constexpr uint8_t controller_to_print = 0;

// Actually drive the motors:
constexpr bool enable_controller = true;

// Use analog pins (A0, A1, A6, A7) instead of (A0, A1, A2, A3), useful for
// saving digital pins on an Arduino Nano:
constexpr bool use_A6_A7 = false;

// Number of faders, must be between 1 and 4:
constexpr size_t num_faders = 4;
// Use phase-correct PWM (true) or fast PWM (false), this determines the timer
// interrupt frequency, prefer phase-correct PWM with prescaler 1 on 16 MHz
// boards, and fast PWM with prescaler 1 on 8 MHz boards, both result in a PWM
// and interrupt frequency of 31.250 kHz (fast PWM is twice as fast):
constexpr bool phase_correct_pwm = true;
// The fader position will be sampled once per `interrupt_counter` timer
// interrupts, this determines the sampling frequency of the control loop:
constexpr uint8_t interrupt_counter = 60 / (1 + phase_correct_pwm);
// The prescaler for the timer, affects PWM and control loop frequencies:
constexpr unsigned prescaler_fac = 1;
// The prescaler for the ADC, affects speed of analog readings:
constexpr uint8_t adc_prescaler_fac = 64;

// Turn off the motor after this many seconds of inactivity:
constexpr float timeout = 2;

// EMA filter factor for fader position filters:
constexpr uint8_t ema_K = 2;

// -------------------------- Computed Quantities --------------------------- //

constexpr auto prescaler0 = factorToTimer0Prescaler(prescaler_fac);
static_assert(prescaler0 != Timer0Prescaler::Invalid, "Invalid prescaler");
constexpr auto prescaler2 = factorToTimer2Prescaler(prescaler_fac);
static_assert(prescaler2 != Timer2Prescaler::Invalid, "Invalid prescaler");
constexpr float Ts = 1. * prescaler_fac * interrupt_counter * 256 *
                     (1 + phase_correct_pwm) / F_CPU;
constexpr float interrupt_freq =
    1. * F_CPU / prescaler_fac / 256 / (1 + phase_correct_pwm);
constexpr auto adc_prescaler = factorToADCPrescaler(adc_prescaler_fac);
static_assert(adc_prescaler != ADCPrescaler::Invalid, "Invalid prescaler");
constexpr float adc_freq = 1. * F_CPU / adc_prescaler_fac;

// --------------------- ADC and Capacitive Touch State --------------------- //

uint16_t ADCReadManual(uint8_t idx);
volatile int16_t adcvals[num_faders]; // Latest ADC reading (updated in ADC ISR)
EMA<ema_K, uint16_t> filters[num_faders]; // Filters for ADC readings
uint16_t filtered_adcval[num_faders];     // Filtered ADC readings

void touchBegin();
volatile bool touched[num_faders]; // Whether the knobs are being touched

// ------------------------------- Controller ------------------------------- //

// The main PID controllers. Need tuning for your specific setup:
template <uint8_t Idx>
struct Controller {
    static PID controller;
};

template <>
PID Controller<0>::controller {
    4,      // Kp: proportional gain
    11,     // Ki: integral gain
    -0.028, // Kd: derivative gain
    Ts,     // Ts: sampling time
    40,     // fc: cutoff frequency of derivative filter (Hz), zero to disable
};
template <>
PID Controller<1>::controller {
    4,      // Kp: proportional gain
    11,     // Ki: integral gain
    -0.028, // Kd: derivative gain
    Ts,     // Ts: sampling time
    40,     // fc: cutoff frequency of derivative filter (Hz), zero to disable
};
template <>
PID Controller<2>::controller {
    4,      // Kp: proportional gain
    11,     // Ki: integral gain
    -0.028, // Kd: derivative gain
    Ts,     // Ts: sampling time
    40,     // fc: cutoff frequency of derivative filter (Hz), zero to disable
};
template <>
PID Controller<3>::controller {
    4,      // Kp: proportional gain
    11,     // Ki: integral gain
    -0.028, // Kd: derivative gain
    Ts,     // Ts: sampling time
    40,     // fc: cutoff frequency of derivative filter (Hz), zero to disable
};

template <uint8_t Idx>
void updateController(int16_t setpoint, int16_t adcval, bool touched) {
    auto &controller = Controller<Idx>::controller;

    // Prevent the motor from being turned off after begin touched
    if (touched) controller.resetActivityCounter();

    // Set the target position
    controller.setSetpoint(setpoint);

    // Update the PID controller to get the control action
    int16_t control = controller.update(adcval);

    // Apply the control action to the motor
    if (enable_controller) {
        if (touched) // Turn off motor if knob is touched
            Motor<Idx>::forward(0);
        else if (control >= 0)
            Motor<Idx>::forward(control);
        else
            Motor<Idx>::backward(-control);
    }

    // Print status
    if (print_controller_signals && Idx == controller_to_print) {
        Serial.print(controller.getSetpoint());
        Serial.print('\t');
        Serial.print(adcval);
        Serial.print('\t');
        Serial.print((control + 256) * 2);
        Serial.println();
    }
}

volatile int16_t setpoints[num_faders];

template <uint8_t Idx>
void readAndUpdateController() {
    int16_t adcval, setpoint;
    // Read the ADC value for the given fader:
    ATOMIC_BLOCK(ATOMIC_FORCEON) { adcval = ::adcvals[Idx]; }
    // If the ADC value was updated by the ADC interrupt, run the control loop:
    if (adcval >= 0) {
        // Check if the fader knob is touched
        bool touched = ::touched[Idx];
        // Read the target position
        ATOMIC_BLOCK(ATOMIC_FORCEON) { setpoint = ::setpoints[Idx]; }
        updateController<Idx>(setpoint, adcval, touched);
        // Write -1 so the controller doesn't run again until the next value is
        // available:
        ATOMIC_BLOCK(ATOMIC_FORCEON) { ::adcvals[Idx] = -1; }
        if (num_faders < 2) cbi(PORTB, 5); // Clear overrun indicator
    }
}

// ------------------------------- Main Loop -------------------------------- //

void setup() {
    for (uint8_t i = 0; i < num_faders; ++i) {
        adcvals[i] = -1;
        filtered_adcval[i] = ADCReadManual(i);
        filters[i].reset(filtered_adcval[i]);
    }

    if (print_frequencies || print_controller_signals) Serial.begin(1000000);

    setupADC(adc_prescaler);
    if (num_faders > 0) setupMotorTimer2(phase_correct_pwm, prescaler2);
    if (num_faders > 2) setupMotorTimer0(phase_correct_pwm, prescaler0);

    if (num_faders > 0) Controller<0>::controller.setActivityTimeout(timeout);
    if (num_faders > 1) Controller<1>::controller.setActivityTimeout(timeout);
    if (num_faders > 2) Controller<2>::controller.setActivityTimeout(timeout);
    if (num_faders > 3) Controller<3>::controller.setActivityTimeout(timeout);

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        if (num_faders < 2) sbi(DDRB, 5); // Pin 13 output (overrun indicator)

        if (num_faders > 0) Motor<0>::begin();
        if (num_faders > 1) Motor<1>::begin();
        if (num_faders > 2) Motor<2>::begin();
        if (num_faders > 3) Motor<3>::begin();

        touchBegin();
    }

    if (print_frequencies) {
        Serial.print(F("Interrupt frequency (Hz): "));
        Serial.println(interrupt_freq);
        Serial.print(F("Controller sampling time (µs): "));
        Serial.println(Ts * 1e6);
    }
    if (print_controller_signals) {
        Serial.println(F("0\t0\t0\t0\r\n0\t0\t0\t1024"));
    }

    // Initalize I²C slave and attach callbacks
    Wire.begin(8);
    void onRequest();
    void onReceive(int);
    Wire.onRequest(onRequest);
    Wire.onReceive(onReceive);

    sbi(TIMSK2, TOIE2); // Enable Timer2 overflow interrupt
}

void loop() {
    if (num_faders > 0) readAndUpdateController<0>();
    if (num_faders > 1) readAndUpdateController<1>();
    if (num_faders > 2) readAndUpdateController<2>();
    if (num_faders > 3) readAndUpdateController<3>();
}

// ---------------------------- Capacitive Touch ---------------------------- //

// Increase this time constant if the capacitive touch sense is too sensitive or
// decrease it if it's not sensitive enough:
constexpr float rc_time_untouched = 200e-6; // seconds

// Compute the actual threshold as a number of interrupts:
constexpr uint8_t touch_sense_thres = interrupt_freq * rc_time_untouched * 2;
// Ignore mains noise by making the “touched” status stick for longer than the
// mains period:
constexpr float period_50Hz = 1. / 50;
constexpr uint8_t touch_sense_stickiness =
    interrupt_freq * period_50Hz * 2 / interrupt_counter;

// Masks of the touch pins (all on port B):
constexpr uint8_t touch_masks[] = {
    1 << 0,
    1 << 1,
    1 << 2,
    1 << 4,
};
constexpr uint8_t touch_mask = (num_faders > 0 ? touch_masks[0] : 0) |
                               (num_faders > 1 ? touch_masks[1] : 0) |
                               (num_faders > 2 ? touch_masks[2] : 0) |
                               (num_faders > 3 ? touch_masks[3] : 0);

void touchBegin() {
    PORTB &= ~touch_mask; // low
    DDRB |= touch_mask;   // output mode
}

// 0. The pin mode is “output”, the value is “low”.
// 1. Set the pin mode to “input”, touch_timer = 0.
// 2. The pin will start charging through the external pull-up resistor.
// 3. After a fixed amount of time, check whether the pin became “high”:
//    if this is the case, the RC-time of the knob/pull-up resistor circuit
//    was smaller than the given threshold. Since R is fixed, this can be used
//    to infer C, the capacitance of the knob: if the capacitance is lower than
//    the threshold (i.e. RC-time is lower), this means the knob was not touched.
// 5. Set the pin mode to “output”, to start discharging the pin to 0V again.
// 6. Some time later, the pin has discharged, so switch to “input” mode and
//    start charging again for the next RC-time measurement.
//
// The “touched” status is sticky: it will remain set for at least
// touch_sense_stickiness ticks. If the pin never resulted in another “touched”
// measurement during that period, the “touched” status for that pin is cleared.

void touchSample(uint8_t counter) {
    static uint8_t touch_timers[num_faders] {};
    if (counter == 0) {
        DDRB &= ~touch_mask; // input mode, start charging
    } else if (counter == touch_sense_thres) {
        uint8_t touched_bits = PINB;
        DDRB |= touch_mask; // output mode, start discharging
        for (uint8_t i = 0; i < num_faders; ++i) {
            bool touch_i = (touched_bits & touch_masks[i]) == 0;
            if (touch_i) {
                touch_timers[i] = touch_sense_stickiness;
                touched[i] = true;
            } else if (touch_timers[i] > 0) {
                --touch_timers[i];
                if (touch_timers[i] == 0) touched[i] = false;
            }
        }
    }
}

// ---------------------------------- ADC ----------------------------------- //

volatile uint8_t adc_mux_idx = num_faders;

constexpr uint8_t adc_mux_idx_to_mux_address(uint8_t adc_mux_idx) {
    return use_A6_A7 ? (adc_mux_idx < 2 ? adc_mux_idx : adc_mux_idx + 4)
                     : adc_mux_idx;
}

void ADCStartConversion(uint8_t channel) {
    adc_mux_idx = channel;
    ADMUX &= 0xF0;
    ADMUX |= adc_mux_idx_to_mux_address(channel);
    sbi(ADCSRA, ADSC); // Start conversion
}

constexpr uint8_t adc_start_count = interrupt_counter / num_faders;
constexpr float adc_rate = interrupt_freq / adc_start_count;
// Check that this doesn't take more time than the 13 ADC clock cycles it takes
// to actually do the conversion.
static_assert(adc_rate <= adc_freq / 14, "ADC too slow");

// Start an ADC conversion at the right intervals.
void ADCSample(uint8_t counter) {
    if (num_faders > 0 && counter == 0 * adc_start_count)
        ADCStartConversion(0);
    else if (num_faders > 1 && counter == 1 * adc_start_count)
        ADCStartConversion(1);
    else if (num_faders > 2 && counter == 2 * adc_start_count)
        ADCStartConversion(2);
    else if (num_faders > 3 && counter == 3 * adc_start_count)
        ADCStartConversion(3);
}

uint16_t ADCReadManual(uint8_t idx) {
    return analogRead(adc_mux_idx_to_mux_address(idx));
}

// ------------------------------- Interrupts ------------------------------- //

ISR(TIMER2_OVF_vect) {
    static uint8_t counter = 0;

    ADCSample(counter);
    touchSample(counter);

    ++counter;
    if (counter == interrupt_counter) counter = 0;
}

ISR(ADC_vect) {
    if (num_faders < 2 && adcvals[adc_mux_idx] >= 0)
        sbi(PORTB, 5);     // Set overrun indicator
    uint16_t result = ADC; // Store ADC reading
    adcvals[adc_mux_idx] = result;
    filtered_adcval[adc_mux_idx] = filters[adc_mux_idx](result << (6 - ema_K));
}

// ---------------------------------- Wire ---------------------------------- //

void onRequest() {
    uint8_t touch = 0;
    if (num_faders > 0) touch |= touched[0] << 0;
    if (num_faders > 1) touch |= touched[1] << 1;
    if (num_faders > 2) touch |= touched[2] << 2;
    if (num_faders > 3) touch |= touched[3] << 3;
    Wire.write(touch);
    for (uint8_t i = 0; i < num_faders; ++i)
        Wire.write(reinterpret_cast<const uint8_t *>(&filtered_adcval[i]), 2);
}

void onReceive(int count) {
    if (count < 2) return;
    if (Wire.available() < 2) return;
    uint16_t data = Wire.read();
    data |= uint16_t(Wire.read()) << 8;
    uint8_t idx = data >> 12;
    data &= 0x03FF;
    if (num_faders > 0 && idx == 0) setpoints[0] = data;
    if (num_faders > 1 && idx == 1) setpoints[1] = data;
    if (num_faders > 2 && idx == 2) setpoints[2] = data;
    if (num_faders > 3 && idx == 3) setpoints[3] = data;
}