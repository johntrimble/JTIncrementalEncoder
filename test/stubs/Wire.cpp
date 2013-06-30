#include "Wire.h"

void TwoWire::begin(uint8_t slaveAddress) { }
void TwoWire::onReceive(void (*f)(void)) { }
void TwoWire::onRequest(void (*f)(void)) { }
void TwoWire::write(uint8_t val) { }
int TwoWire::read() { return 0; }
