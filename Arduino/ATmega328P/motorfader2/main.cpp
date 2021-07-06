// Configuration and initialization of the analog-to-digital converter:
#include "ADC.hpp"
// PID controller:
#include "Controller.hpp"
// Configuration of PWM and Timer0 for driving the motor:
#include "Motor.hpp"
// Reference signal for testing the performance of the controller:
#include "Reference.hpp"
// Helpers for low-level AVR Timer0 and ADC registers
#include "Registers.hpp"
// Capacitive touch sensing
#include "Touch.hpp"

#include <Arduino.h>

// ------------------------------ Description ------------------------------- //

// This sketch drives a motorized fader using a PID controller. It follows a
// reference trajectory defined in `Reference.hpp`. The motor is disabled when
// the user touches the knob of the fader.
//
// Everything is driven by Timer0, which runs (by default) at a rate of
// 31.250 kHz. This high rate is used to eliminate audible tones from the PWM
// drive for the motor.
// Every 30 periods of Timer0 (960 µs), the analog input is sampled, and
// this causes the PID control loop to run in the main loop function.
// Capacitive sensing is implemented by measuring the RC time on the touch pin
// in the Timer0 interrupt handler. The “touched” status is sticky for >20 ms
// to prevent interference from the 50 Hz mains.

// -------------------------------- Hardware -------------------------------- //

// A0:  potentiometer of motorized fader (ADMUX0)
// D2:  touch pin of motorized fader     (PD2)
// D4:  input 1A of L293D dual H-bridge  (PD4)
// D5:  input 2A of L293D dual H-bridge  (OC0B)
// D13: scope to monitor timing          (PB5)
//
// Connect the outer connections of the potentiometer to ground and Vcc, it's
// recommended to add a 100 nF capacitor between the wiper and ground.
// Connect the 1,2EN enable pin of the L293D to Vcc.
// Connect a 500kΩ pull-up resistor between pin D2 and Vcc.

// ----------------------------- Configuration ------------------------------ //

// Print the control loop and interrupt frequencies to Serial at startup:
constexpr bool PRINT_FREQUENCIES = false;

// Print the setpoint, actual position and control signal to Serial.
// Note that this slows down the control loop significantly, it goes from
// 29% to >83% CPU usage.
constexpr bool PRINT_CONTROLLER_SIGNALS = true;
constexpr uint8_t controller_to_print = 2;

// Actually drive the motors:
constexpr bool ENABLE_CONTROLLER = true;

// Number of faders. Must be between 1 and 4.
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

// -------------------------- Computed Quantities --------------------------- //

constexpr auto prescaler0 = factorToTimer0Prescaler(prescaler_fac);
static_assert(prescaler0 != Timer0Prescaler::Invalid, "Invalid prescaler");
constexpr auto prescaler2 = factorToTimer2Prescaler(prescaler_fac);
static_assert(prescaler2 != Timer2Prescaler::Invalid, "Invalid prescaler");
constexpr float Ts = 1. * prescaler_fac * interrupt_counter * 256 *
                     (1 + phase_correct_pwm) / F_CPU;
constexpr float interrupt_freq =
    1. * F_CPU / prescaler_fac / 256 / (1 + phase_correct_pwm);

// ---------------------------------- Main ---------------------------------- //

void setup();
void loop();
int main() {
    setup();
    while (true) {
        loop();
    }
}

volatile int16_t adcval[num_faders]; // Latest ADC reading (updated in ADC ISR)

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

// Activation function converts controller output to a PWM duty cycle (0-255)
const uint8_t knee = 0; // set to positive value to eliminate PWM dead-zone
uint8_t activation(uint8_t val) {
    return val == 0 ? 0 : map(val, 0, 255, knee, 255);
}

template <uint8_t Idx>
void updateController(int16_t adcval, bool touched) {
    auto &controller = Controller<Idx>::controller;
    // Get the next setpoint
    static uint8_t counter = 0;
    if (counter == 0)
        controller.setSetpoint(getNextSetpoint<Idx>());
    if (counter++ >= 8)
        counter = 0;

    // Update the PID controller to get the control action
    int16_t control = controller.update(adcval);

    // Apply the control action to the motor
    if (ENABLE_CONTROLLER) {
        if (touched) // Turn off motor if knob is touched
            Motor<Idx>::forward(0);
        else if (control >= 0)
            Motor<Idx>::forward(activation(control));
        else
            Motor<Idx>::backward(activation(-control));
    }

    // Print status
    if (PRINT_CONTROLLER_SIGNALS && Idx == controller_to_print) {
        // Serial.print(controller.getSetpoint());
        // Serial.print('\t');
        // Serial.print(adcval);
        // Serial.print('\t');
        Serial.println((control + 256) * 2);
    }
}

template <uint8_t Idx>
void readAndUpdateController() {
    int16_t adcval;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { adcval = ::adcval[Idx]; }
    bool touched = TouchSense<Idx>::touched;
    if (adcval >= 0) {
        if (Idx == 0)
            sbi(PORTB, 5);
        ATOMIC_BLOCK(ATOMIC_FORCEON) { ::adcval[Idx] = -1; }
        updateController<Idx>(adcval, touched);
        if (Idx == num_faders - 1)
            cbi(PORTB, 5);
    }
}

// ------------------------------- Main Loop -------------------------------- //

void setup() {
    for (uint8_t i = 0; i < num_faders; ++i)
        adcval[i] = -1;

    if (PRINT_FREQUENCIES || PRINT_CONTROLLER_SIGNALS)
        Serial.begin(1000000);

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        sbi(DDRB, 5); // pin 13 output (for debugging the timing)

        if (num_faders > 0)
            Motor<0>::begin();
        if (num_faders > 1)
            Motor<1>::begin();
        if (num_faders > 2)
            Motor<2>::begin();
        if (num_faders > 3)
            Motor<3>::begin();

        if (num_faders > 0)
            TouchSense<0>::begin();
        if (num_faders > 1)
            TouchSense<1>::begin();
        if (num_faders > 2)
            TouchSense<2>::begin();
        if (num_faders > 3)
            TouchSense<3>::begin();
    }

    setupADC();
    if (num_faders > 0)
        setupMotorTimer0(phase_correct_pwm, prescaler0);
    if (num_faders > 2)
        setupMotorTimer2(phase_correct_pwm, prescaler2);

    if (PRINT_FREQUENCIES) {
        Serial.print(F("Interrupt frequency (Hz): "));
        Serial.println(interrupt_freq);
        Serial.print(F("Controller sampling time (µs): "));
        Serial.println(Ts * 1e6);
    }
    if (PRINT_CONTROLLER_SIGNALS) {
        Serial.println(F("0\t0\t0\t0\r\n0\t0\t0\t1024"));
    }

    sbi(TIMSK0, TOIE0); // Enable timer 0 overflow interrupt
}

void loop() {
    if (num_faders > 0)
        readAndUpdateController<0>();
    if (num_faders > 1)
        readAndUpdateController<1>();
    if (num_faders > 2)
        readAndUpdateController<2>();
    if (num_faders > 3)
        readAndUpdateController<3>();
}

// ---------------------------- Capacitive Touch ---------------------------- //

// Increase this time constant if the capacitive touch sense is too sensitive or
// decrease it if it's not sensitive enough:
constexpr float rc_time_untouched = 200e-6; // seconds

// Compute the actual threshold as a number of interrupts:
extern const int16_t touch_sense_thres =
    interrupt_freq * rc_time_untouched * 2 / num_faders;
// Ignore mains noise by making the “touched” status stick for longer than the
// mains period:
constexpr float period_50Hz = 1. / 50;
extern const uint16_t touch_sense_stickiness =
    interrupt_freq * period_50Hz * 1.2 / num_faders;
// How many interrupts to discharge the pin for:
extern const int16_t touch_sense_discharge = 10 / num_faders;

void touchSample() {
    static uint8_t counter = 0;

    if (num_faders > 0 && counter == 0)
        TouchSense<0>::update();
    if (num_faders > 1 && counter == 1)
        TouchSense<1>::update();
    if (num_faders > 2 && counter == 2)
        TouchSense<2>::update();
    if (num_faders > 3 && counter == 3)
        TouchSense<3>::update();

    counter++;
    if (counter >= num_faders)
        counter = 0;
}

// ---------------------------------- ADC ----------------------------------- //

volatile uint8_t adc_mux_idx = num_faders;

void ADCNextChannel() {
    ++adc_mux_idx;
    if (adc_mux_idx < num_faders) {
        ADMUX &= 0xF0;
        ADMUX |= adc_mux_idx;
    }
}

// Enable the ADC trigger every `interrupt_counter` calls.
void ADCSample() {
    static uint8_t counter = 0;
    counter++;
    if (counter >= interrupt_counter) {
        counter = 0;
        if (adc_mux_idx == num_faders) {
            adc_mux_idx = 0;
            ADMUX &= 0xF0;
            sbi(ADCSRA, ADSC); // Start conversion
        }
    }
}

// ------------------------------- Interrupts ------------------------------- //

ISR(TIMER0_OVF_vect) {
    ADCSample();
    touchSample();
}

ISR(ADC_vect) {
    adcval[adc_mux_idx] = ADC; // Store ADC reading
    ADCNextChannel();
    if (adc_mux_idx < num_faders)
        sbi(ADCSRA, ADSC); // Start conversion
}
