#ifndef Arduino_h
#define Arduino_h

#include <vector>
#include "inttypes.h"

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define true 0x1
#define false 0x0

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))
#define max(X, Y)  ((X) > (Y) ? (X) : (Y))

#define lowByte(X) ((uint8_t)X)
#define highByte(X) ((uint8_t)((X) >> 8))
#define word(H,L) ((uint16_t)(((H) << 8)|L))

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define HIGH 0x1
#define LOW  0x0

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

void pinMode(uint8_t pin, uint8_t mode);

int analogRead(uint8_t pin);

void detachInterrupt(uint8_t);

void attachInterrupt(uint8_t, void (*)(void), int mode);

int map(int value, int sourceStart, int sourceEnd, int destStart, int destEnd);

unsigned long micros();

void delay(unsigned long);

class Print {
public:
  inline size_t print(const char[]) { return 0; }
  inline size_t print(char) { return 0; }
  inline size_t print(unsigned char, int = DEC) { return 0; }
  inline size_t print(int, int = DEC) { return 0; }
  inline size_t print(unsigned int, int = DEC) { return 0; }
  inline size_t print(long, int = DEC) { return 0; }
  inline size_t print(unsigned long, int = DEC) { return 0; }
  inline size_t print(double, int = 2) { return 0; }

  inline size_t println(const char[]) { return 0; }
  inline size_t println(char) { return 0; }
  inline size_t println(unsigned char, int = DEC) { return 0; }
  inline size_t println(int, int = DEC) { return 0; }
  inline size_t println(unsigned int, int = DEC) { return 0; }
  inline size_t println(long, int = DEC) { return 0; }
  inline size_t println(unsigned long, int = DEC) { return 0; }
  inline size_t println(double, int = 2) { return 0; }
  inline size_t println(void) { return 0; }
};

class HardwareSerial : public Print {

};

extern HardwareSerial Serial;

#endif
