#include "Arduino.h"
#include "jt_pin.h"

namespace JTIncrementalEncoder {

AnalogOutputPin::AnalogOutputPin(int pin) 
  : _pin(pin) {
  pinMode(pin, OUTPUT);
}

void AnalogOutputPin::write(uint8_t value) {
  analogWrite(this->_pin, value);
}

DigitalInputPin::DigitalInputPin(int pin) 
  : _pin(pin) {
  pinMode(pin, INPUT);
}

uint8_t DigitalInputPin::read() {
  return digitalRead(this->_pin);
}

}
