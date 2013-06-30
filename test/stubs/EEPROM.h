#ifndef EEPROM_h
#define EEPROM_h

#include <vector>
#include <inttypes.h>

class EEPROMClass
{
  std::vector<uint8_t> buffer;

  public:
    EEPROMClass(size_t buffer_size): 
      buffer(buffer_size) {}
    uint8_t read(int addr);
    void write(int addr, uint8_t data);
    std::vector<uint8_t>& get_buffer();
};

#endif