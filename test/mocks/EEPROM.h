#ifndef EEPROM_h
#define EEPROM_h

#include <vector>
#include <inttypes.h>

class EEPROMMock
{
  std::vector<uint8_t> buffer;

  public:
    EEPROMMock(size_t buffer_size): 
      buffer(buffer_size) {}
    uint8_t read(int addr);
    void write(int addr, uint8_t data);
    std::vector<uint8_t>& get_buffer();
};

extern EEPROMMock EEPROM;

#endif