#ifndef Arduino_h
#define Arduino_h

#include <vector>
#include "inttypes.h"

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))
#define max(X, Y)  ((X) > (Y) ? (X) : (Y))

#define lowByte(X) ((uint8_t)X)
#define highByte(X) ((uint8_t)((X) >> 8))
#define word(H,L) ((uint16_t)(((H) << 8)|L))

static const int NUMBER_DIGITAL_PINS = 14;
static const int NUMBER_ANALOG_PINS = 6;

class PinMock {
  std::vector<std::vector<uint8_t> > digital_values;
  std::vector<std::vector<int> > analog_values;
  std::vector<std::vector<uint8_t> > written_analog_values;
  std::vector<std::vector<uint8_t> > written_digital_values;
public:
  PinMock(): 
    digital_values(NUMBER_DIGITAL_PINS),
    analog_values(NUMBER_ANALOG_PINS), 
    written_analog_values(NUMBER_ANALOG_PINS),
    written_digital_values(NUMBER_DIGITAL_PINS) {}
  virtual void set_digital_value_sequence(uint8_t pin, std::vector<uint8_t> &values);
  virtual uint8_t next_digital_value(uint8_t pin);
  virtual void set_analog_value_sequence(uint8_t pint, std::vector<int> &values);
  virtual uint8_t next_analog_value(uint8_t pin);
  virtual void write_analog_value(uint8_t pin, uint8_t value);
  virtual void write_digital_value(uint8_t pin, uint8_t value);
  virtual std::vector<uint8_t>& get_written_analog_values(uint8_t pin);
  virtual std::vector<uint8_t>& get_written_digital_values(uint8_t pin);
  virtual void clear();
private:
  uint8_t pop(std::vector<uint8_t> &l);
  int pop(std::vector<int> &l);
};

extern PinMock pinMock;

uint8_t digitalRead(uint8_t pin);

void digitalWrite(uint8_t pin, uint8_t value);

int analogRead(uint8_t pin);

void detachInterrupt(uint8_t);

#endif
