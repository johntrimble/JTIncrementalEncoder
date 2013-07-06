#ifndef Wire_h
#define Wire_h

#include <inttypes.h>
#include <stdio.h>

class TwoWire {
public:
  void begin(uint8_t slaveAddress);
  void onReceive(void (*f)(int bytesReceived));
  void onRequest(void (*f)(void));
  void write(uint8_t val);
  size_t write(const uint8_t *buffer, size_t size);
  int read();
};

#endif
