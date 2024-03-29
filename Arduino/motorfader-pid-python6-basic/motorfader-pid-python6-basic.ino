#include "PID.hpp"
#include "SerialInput.hpp"
#include "Hysteresis.hpp"

const uint8_t signal[] PROGMEM = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,
  127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
  200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
  200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
  200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
  
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
  127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,

  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

#define LEN(x) (sizeof(x) / sizeof(*x))

const size_t len = LEN(signal);

const unsigned long Ts = 1000; // microseconds

const uint16_t forward = 9;
const uint16_t backward = 10;
const uint16_t enable = 4;

/*PID pid = {
  0.6,
  0.00,
  0.0001,
  Ts * 1e-6 // seconds
};*/

PID pid = {
  0,
  0,
  0,
  Ts * 1e-6 // seconds
};

constexpr uint8_t HYST_BITS = 2;

EMA_f ema = 0.8;
Hysteresis<HYST_BITS> hyst;

StreamInput<16> serIn(Serial);

void setup() {
  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);
  Serial.begin(115200);
  pid.set(64);
  
  // 1 → /1
  // 2 → /8
  // 3 → /64
  // 4 → /256
  // 5 → /1024
  TCCR1B = TCCR1B & 0b11111000 | 1 ;
}

const int16_t knee = 30 * 2;
// const int16_t knee = 40;

uint8_t activation(int16_t val) {
  if (val == 0)
    return 0;
  else
    return map(val, 0, 255, knee, 255);
}

void writeMotor(int16_t val) {
  if (val >= 0) {
    analogWrite(forward, activation(val));
    digitalWrite(backward, 0);
  } else {
    analogWrite(backward, activation(-val));
    digitalWrite(forward, 0);
  }
}

void print(int16_t target, int16_t val, int16_t output) {
  Serial.print(target); Serial.print('\t');
  Serial.print(val); Serial.print('\t');
  Serial.println(output);
  Serial.flush();
}

const char EOT = 0x04;
const char ACK = 0x06;

void loop() {
  char serVal = serIn.check();
  switch (serVal) {
    case 'p':
      pid.setProp(serIn.getValue());
      Serial.print(ACK);
      break;
    case 'i':
      pid.setInt(serIn.getValue());
      Serial.print(ACK);
      break;
    case 'd':
      pid.setDeriv(serIn.getValue());
      Serial.print(ACK);
      break;
    case 'g':
      Serial.print(ACK);
      run();
      Serial.print(EOT);
      break;
  }
}

void run() {
  unsigned long prevMicros = micros();
  size_t index = 0;
  bool running = true;
  size_t counter = 0;

  while (running) {
    if (micros() - prevMicros >= Ts) {
      if (counter >= 10) {
        counter = 0;
      }
      if (counter == 0) {
        pid.set(pgm_read_byte(signal + index) * 4);
        index++;
        running = index < len;
      }
      hyst.update(ema(analogRead(A0)));
      auto output = hyst.getValue() << HYST_BITS;
      int16_t control = pid.update(output);
      writeMotor(control);
      print(pid.getSetPoint(), control, output);
      prevMicros += Ts;
      counter++;
    }
  }
  writeMotor(0);
}
