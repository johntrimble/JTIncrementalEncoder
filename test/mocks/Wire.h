#ifndef Wire_h
#define Wire_h

#include <inttypes.h>

class TwoWire {
  void begin(uint8_t slaveAddress);
  void onReceive(void (*f)(void));
  void onRequest(void (*f)(void));
  void write(uint8_t val);
  int read();
};

extern TwoWire Wire;

#endif
