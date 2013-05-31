#include<MCP42xxx.h>

typedef struct {
  int inputPin;
  int rawInputPin;
  int interrupt;
  int average;
  int minValue;
  int maxValue;
  MCP42xxx::Channel channel;
  void (*isrFunc)(void);
} EncoderChannel;


typedef struct {
  byte status;
  byte error;
  byte state;
  byte position;
  byte mode;
  byte divisions;
  byte identification;
} EncoderRegisters;
