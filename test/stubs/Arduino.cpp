#include <list>
#include <algorithm>
#include "Arduino.h"

void PinMock::set_digital_value_sequence(uint8_t pin, std::vector<uint8_t> &values) {
  digital_values[pin] = values;
}

void PinMock::set_analog_value_sequence(uint8_t pin, std::vector<int> &values) {
  analog_values[pin] = values;
}

uint8_t PinMock::next_digital_value(uint8_t pin) {
  return pop(digital_values[pin]);
}

uint8_t PinMock::next_analog_value(uint8_t pin) {
  return pop(analog_values[pin]);
}

void PinMock::write_analog_value(uint8_t pin, uint8_t value) {
  written_analog_values[pin].push_back(value);
}

void PinMock::write_digital_value(uint8_t pin, uint8_t value) {
  written_digital_values[pin].push_back(value);
}

std::vector<uint8_t>& PinMock::get_written_analog_values(uint8_t pin) {
  return written_analog_values[pin];
}

std::vector<uint8_t>& PinMock::get_written_digital_values(uint8_t pin) {
  return written_digital_values[pin];
}

void PinMock::clear() {
  for(unsigned int i = 0; i < digital_values.size(); i++ ) {
    digital_values[i].clear();
  }
  for(unsigned int i = 0; i < analog_values.size(); i++ ) {
    analog_values[i].clear();
  }
}

uint8_t PinMock::pop(std::vector<uint8_t> &l) {
  uint8_t rvalue = l.front();
  l.erase(l.begin());
  return rvalue;
}

int PinMock::pop(std::vector<int> &l) {
  int rvalue = l.front();
  l.erase(l.begin());
  return rvalue;
}

PinMock pinMock = PinMock();

uint8_t digitalRead(uint8_t pin) {
  return pinMock.next_digital_value(pin);
}

void digitalWrite(uint8_t pin, uint8_t value) {
  pinMock.write_digital_value(pin, value);
}

void pinMode(uint8_t pin, uint8_t mode) {}

int analogRead(uint8_t pin) {
  return pinMock.next_analog_value(pin);
}

void detachInterrupt(uint8_t interrupt) {

}

void attachInterrupt(uint8_t interrupt, void (*f)(void), int mode) { 

}

int map(int value, int sourceStart, int sourceEnd, int destStart, int destEnd) {
  return (value * ((float)(destEnd-destStart) / (float)(sourceEnd-sourceStart))) + destStart;
}

static unsigned long currentMicros = 0;
unsigned long micros() {
  return currentMicros;
}

void delay(unsigned long) {
  
}

HardwareSerial Serial;