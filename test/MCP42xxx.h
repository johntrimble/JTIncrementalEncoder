#ifndef MCP42xxx_h
#define MCP42xxx_h

class MCP42xxx
{
  public:
    enum Channel { NONE=0, CHANNEL_0=1, CHANNEL_1=2, BOTH=3 };
    void write(Channel channel, uint8_t value);
};

#endif
