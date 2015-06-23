#ifndef jt_pin_h
#define jt_pin_h

#include <inttypes.h>

namespace JTIncrementalEncoder {

class AnalogOutputPin {
  private:
  int _pin;

  public:
  AnalogOutputPin(int pin);
  void write(uint8_t value);
};

class DigitalInputPin {
  private:
  int _pin;

  public:
  DigitalInputPin(int pin);
  uint8_t read();
};

}

#endif
