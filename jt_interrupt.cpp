#include "Arduino.h"
#include "jt_interrupt.h"

namespace JTIncrementalEncoder {

InterruptHandler::~InterruptHandler() { }
  
Interrupt::Interrupt(int interruptNumber, void (*f)()) 
    : _interruptNumber(interruptNumber), 
      _handler(NULL), 
      _isr(f), 
      _mode(CHANGE) { }

void Interrupt::disable() {
  detachInterrupt(this->_interruptNumber);
}

void Interrupt::enable() {
  attachInterrupt(this->_interruptNumber, this->_isr, this->_mode);
}

void Interrupt::setInterruptHandler(InterruptHandler* handler, int mode) {
  InterruptHandler* oldHandler = this->_handler;
  // detach interrupt if handler is NULL
  this->_handler = handler;
  this->_mode = mode;
  if( handler == NULL ) {
    this->disable();
  } else {
    this->enable();
  }
}

void Interrupt::fireInterruptOccurred() {
  if( this->_handler != NULL ) {
    _handler->interruptOccurred(); 
  }
}

static void interrupt0ISR();
static void interrupt1ISR();

Interrupt interrupt0(0, interrupt0ISR);
Interrupt interrupt1(1, interrupt1ISR);


static void interrupt0ISR() {
  interrupt0.fireInterruptOccurred();
}

static void interrupt1ISR() {
  interrupt1.fireInterruptOccurred();
}

}
