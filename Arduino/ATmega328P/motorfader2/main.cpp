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
// Connect the outer connections of the potentiometer to ground and Vcc.
// Connect the 1,2EN enable pin of the L293D to Vcc.
// Connect a 500kΩ pull-up resistor between pin D2 and Vcc.

// ----------------------------- Configuration ------------------------------ //

// Print the control loop and interrupt frequencies to Serial at startup:
constexpr bool PRINT_FREQUENCIES = true;

// Print the setpoint, actual position and control signal to Serial.
// Note that this slows down the control loop significantly, it goes from
// 29% to >83% CPU usage.
constexpr bool PRINT_CONTROLLER_SIGNALS = false;

// Actually drive the motors:
constexpr bool ENABLE_CONTROLLER = true;

// Analog channel (0-5 on Uno, 0-7 on Nano)
constexpr uint8_t analog_input = 0;
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

constexpr auto prescaler = factorToTimer0Prescaler(prescaler_fac);
static_assert(prescaler != Timer0Prescaler::Invalid, "Invalid prescaler");
constexpr float Ts = 1. * prescaler_fac * interrupt_counter * 256 *
                     (1 + phase_correct_pwm) / F_CPU;
constexpr float interrupt_freq =
    1. * F_CPU / prescaler_fac / 256 / (1 + phase_correct_pwm);

// ---------------------------------- Main ---------------------------------- //

int main() {
    if (PRINT_FREQUENCIES || PRINT_CONTROLLER_SIGNALS)
        Serial.begin(2000000);

    setupMotorTimer(phase_correct_pwm, prescaler);
    setupADC(analog_input);

    if (PRINT_FREQUENCIES) {
        Serial.print(F("Interrupt frequency (Hz): "));
        Serial.println(interrupt_freq);
        Serial.print(F("Controller sampling time (µs): "));
        Serial.println(Ts * 1e6);
    }
    if (PRINT_CONTROLLER_SIGNALS) {
        Serial.println(F("0\t0\t0\t0\r\n0\t0\t0\t1024"));
    }

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        sbi(DDRB, 5);       // pin 13 output
        cbi(PORTD, 2);      // pin 2 low
        sbi(DDRD, 2);       // pin 2 output
        sbi(TIMSK0, TOIE0); // Enable timer 0 overflow interrupt
    }
    while (true) {
        loop();
    }
}

// ------------------------------- Controller ------------------------------- //

// The main PID controller. Needs tuning for your specific setup:
PID controller = {
    4,       // Kp: proportional gain
    11,      // Ki: integral gain
    -2.8e-2, // Kd: derivative gain
    Ts,      // Ts: sampling time
    40,      // fc: cutoff frequency of derivative filter (Hz), zero to disable
};

// Activation function converts controller output to a PWM duty cycle (0-255)
const uint8_t knee = 0; // set to positive value to eliminate PWM dead-zone
uint8_t activation(uint8_t val) {
    return val == 0 ? 0 : map(val, 0, 255, knee, 255);
}

void updateController(int16_t adcval, bool touched) {
    static uint8_t counter = 0;
    if (counter == 0)
        controller.setSetpoint(getNextSetpoint());
    if (counter++ >= 8)
        counter = 0;

    int16_t control = controller.update(adcval);

    if (ENABLE_CONTROLLER) {
        if (touched) // Turn of motor if knob is touched
            motorForward(0);
        else if (control >= 0)
            motorForward(activation(control));
        else
            motorBackward(activation(-control));
    }

    if (PRINT_CONTROLLER_SIGNALS) {
        Serial.print(controller.getSetpoint());
        Serial.print('\t');
        Serial.print(adcval);
        Serial.print('\t');
        Serial.println((control + 256) * 2);
    }
}

// ------------------------------- Main Loop -------------------------------- //

volatile int16_t adcval = -1;  // Latest ADC reading (updated in ADC ISR)
volatile bool touched = false; // Knob touch status (updated in Timer0 ISR)

void loop() {
    int16_t adcval;
    ATOMIC_BLOCK(ATOMIC_FORCEON) { adcval = ::adcval; }
    bool touched = ::touched;
    if (adcval >= 0) {
        ATOMIC_BLOCK(ATOMIC_FORCEON) { ::adcval = -1; }
        updateController(adcval, touched);
        cbi(PORTB, 5);
    }
}

// ---------------------------- Capacitive Touch ---------------------------- //

// Increase this time constant if the capacitive touch sense is too sensitive or
// decrease it if it's not sensitive enough:
constexpr float rc_time_untouched = 100e-6; // seconds
constexpr int16_t touch_thres = interrupt_freq * rc_time_untouched * 1.2;
constexpr float period_50Hz = 1. / 50; // ignore mains noise
constexpr uint16_t touch_stickiness = interrupt_freq * period_50Hz * 1.2;
constexpr int16_t touch_discharge = 10;

[[nodiscard]] bool touch_pin_read() { return (PIND & (1 << 2)) != 0; }
[[nodiscard]] bool touch_pin_is_input() { return (DDRD & (1 << 2)) == 0; }
void touch_pin_output() { sbi(DDRD, 2); }
void touch_pin_input() { cbi(DDRD, 2); }

void touchSample() {
    static int16_t touchcounter = 0;
    static uint16_t stickytouched = 0;
    if (touch_pin_is_input()) { // pin 2 is input
        if (touch_pin_read()) { // pin 2 is high
            if (touchcounter > touch_thres) {
                touched = true;
                stickytouched = touch_stickiness;
            }
            touch_pin_output(); // output mode, start discharging
            touchcounter = -touch_discharge;
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

// ---------------------------------- ADC ----------------------------------- //

// Enable the ADC trigger every `interrupt_counter` calls.
void ADCSample() {
    static uint8_t counter = 0;
    counter++;
    if (counter >= interrupt_counter) {
        counter = 0;
        sbi(PORTB, 5);
        sbi(ADCSRA, ADATE); // auto trigger enable
    }
}

// ------------------------------- Interrupts ------------------------------- //

ISR(TIMER0_OVF_vect) {
    ADCSample();
    touchSample();
}

ISR(ADC_vect) {
    adcval = ADC;       // Store ADC reading
    cbi(ADCSRA, ADATE); // Auto trigger disable
}
