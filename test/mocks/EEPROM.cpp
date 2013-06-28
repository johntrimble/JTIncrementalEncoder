#include <vector>
#include <inttypes.h>
#include "EEPROM.h"

uint8_t EEPROMClass::read(int addr) {
  return buffer[addr];
}

void EEPROMClass::write(int addr, uint8_t data) {
  buffer[addr] = data;
}

std::vector<uint8_t>& EEPROMClass::get_buffer() {
  return buffer;
}

EEPROMClass EEPROM(2048);