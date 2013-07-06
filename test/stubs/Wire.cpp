#include "Wire.h"

void TwoWire::begin(uint8_t slaveAddress) { }
void TwoWire::onReceive(void (*f)(int bytesReceived)) { }
void TwoWire::onRequest(void (*f)(void)) { }
void TwoWire::write(uint8_t val) { }
int TwoWire::read() { return 0; }
size_t write(const uint8_t *buffer, size_t size) { return size; }
