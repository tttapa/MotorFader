#pragma once

template <size_t DIGITS>
class StreamInput {
  public:
    StreamInput(Stream &s) : s(s) {}
    char check() {
      while (Serial.available()) {
        char input = Serial.read();
        if (input == '\n') {
          if (index == 0)
            continue;
          buff[index] = '\0';
          index = 0;
          parse();
          return type;
        }
        if (input >= 'a' && input <= 'z' && input != 'e' && input != 'E') {
          type = input;
          continue;
        }
        if (index == DIGITS)
          continue;
        if ((input < '0' || input > '9') && input != 'e' && input != 'E' && input != '.' && input != '-')
          continue;
        
        buff[index++] = input;
      }
      return '\0';
    }

    float getValue() {
      return value;
    }

  private:
    Stream &s;
    
    char buff[DIGITS+1];
    size_t index = 0;
    float value = 0;
    char type = '\0';
    void parse() {
      value = atof(buff);
    }
};

