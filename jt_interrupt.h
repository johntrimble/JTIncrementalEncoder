#ifndef jt_interrupt_h
#define jt_interrupt_h

namespace JTIncrementalEncoder {

class InterruptHandler {
  public:
  virtual void interruptOccurred() = 0;
  virtual ~InterruptHandler() { };
};

class Interrupt {
  private:
  InterruptHandler* volatile _handler;
  void (*_isr)();
  int _interruptNumber;
  int _mode;
  void fireInterruptOccurred();
  
  public:
  Interrupt(int interruptNumber, void (*f)());
  void disable();
  void enable();
  void setInterruptHandler(InterruptHandler* handler, int mode);
};

extern Interrupt interrupt0;
extern Interrupt interrupt1;

}

#endif