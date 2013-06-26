#include <vector>
#include <inttypes.h>
#include "EEPROM.h"

uint8_t EEPROMMock::read(int addr) {
  return buffer[addr];
}

void EEPROMMock::write(int addr, uint8_t data) {
  buffer[addr] = data;
}

std::vector<uint8_t>& EEPROMMock::get_buffer() {
  return buffer;
}

EEPROMMock EEPROM(2048);