#include<EEPROM.h>
#include<SPI.h>
#include<Wire.h>
#include "JTIncrementalEncoder.h"
#include<MCP42xxx.h>

MCP42xxx pot(10, -1, -1);

// General encoder constants
static const int SLAVE_ADDRESS = 31;
static const int DIVISIONS = 92;
static const int NUMBER_CHANNELS = 2;
static const int CALIBRATION_PIN = 6;
static const int RESET_PIN = 8;
static const int INDEX_INDICATOR_PIN = 7;

// Channel A
static const int CHANNEL_A_INPUT_PIN = 2;
static const int CHANNEL_A_INTERRUPT = 0;
static const int CHANNEL_A_RAW_INPUT_PIN = A0;

// Channel B
static const int CHANNEL_B_INPUT_PIN = 3;
static const int CHANNEL_B_INTERRUPT = 1;
static const int CHANNEL_B_RAW_INPUT_PIN = A1;

// Direction
static const byte CLOCKWISE = 0;
static const byte COUNTERCLOCKWISE = 1;

/* I2C interface
 * Control register...
 * Status: Ready, Busy
 * Error: ...
 * State: ...
 * Position: ...
 * Modes: Calibrating, Reset Home Position, Tracking
 * Divisions: ...
 * Identificaion: ...
 */
static const byte NUMBER_REGISTERS = 7;

static const byte CALIBRATION_MODE = 1;
static const byte RESET_HOME_MODE = 2;
static const byte TRACKING_MODE = 3;

static const byte READY_STATUS = 0;
static const byte BUSY_STATUS = 1;
static const byte UNCALIBRATED_STATUS = 2;

static const int STATUS_REGISTER = 0;
static const int ERROR_REGISTER = 1;
static const int STATE_REGISTER = 2;
static const int POSITION_REGISTER = 3;
static const int MODE_INDEX = 4;
static const int DIVISION_REGISTER = 5;
static const int IDENTIFICATION_REGISTER = 6;

static const byte MAX_WRITE_BYTES = 2;

static volatile byte bytesReceivedFromMaster = 0;
static byte receivedData[MAX_WRITE_BYTES];
static byte registers[NUMBER_REGISTERS];
static byte isCalibrated = 0;


// Interrupt vars
static byte isrSteps = 0;

// Current encoder reading, format: 
// - First 4 bits are a counter incremented by one each time the state is updated. 
// - 5th bit is the state of channel A
// - 6th bit is the state of channel B
// - 7th bit is the previous state of channel A
// - 8th bit is the previous state of channel B
static volatile byte encoderState = 0;

// Current position
int currentPosition = 0;

static EncoderChannel channels[NUMBER_CHANNELS];

/**
 * Interrupt handlers for channel A and B. Updates encoderState.
 */
static void channelAISR() {
  encoderState = (encoderState << 2)&B11000000|encoderState&B00100000|(digitalRead(channels[0].inputPin)<<4)|isrSteps&B00001111;
  isrSteps++;
}

static void channelBISR() {
  encoderState = (encoderState << 2)&B11000000|encoderState&B00010000|(digitalRead(channels[1].inputPin)<<5)|isrSteps&B00001111;
  isrSteps++;
}

/**
 * Given the encoder state, returns the current direction.
 */
static byte getDirection(byte encoderState) {
  byte graycode = encoderState&B11110000;
  switch(graycode) {
    case B11100000:
    case B10000000:
    case B00010000:
    case B01110000:
      return CLOCKWISE;
      break;
    default:
      return COUNTERCLOCKWISE;
  }
}

/**
 * Reads from the given analog pin multiple times and returns the mean.
 */
static int sample(int pin) {
  int readings[5];
  for( int i = 0; i < 5; i++ ) {
    readings[i] = analogRead(pin);
  }
  isort(readings, 5);
  return readings[2];
}

/**
 * Sorts the given array of integers using insertion sort.
 */
static void isort(int arr[], int length) {
  for( int i = 1; i < length; i++ ) {
    int value = arr[i];
    int k = i;
    while( k > 0 && value < arr[k-1] ) {
       arr[k] = arr[k-1];
       k--;      
    }
    arr[k] = value;
  }
}

void initializeRegisters() {
  registers[STATUS_REGISTER] = READY_STATUS;
  registers[ERROR_REGISTER] = 0;
  registers[STATE_REGISTER] = encoderState;
  registers[POSITION_REGISTER] = currentPosition;
  registers[MODE_INDEX] = TRACKING_MODE;
  registers[DIVISION_REGISTER] = DIVISIONS;
  registers[IDENTIFICATION_REGISTER] = 0;
}

/**
 * I2C handler for when master is requesting data from this slave.
 */
void requestEvent() {
  Wire.write(registers + receivedData[0], NUMBER_REGISTERS - receivedData[0]);
}

/**
 * I2C handler for when master is writing data to this slave. Format is as follows:
 * - First byte is always the register address byte.
 * - Remaining bytes are to be written to registers, starting at the provided address.
 * If more data is provided than will fit in to the write registers, the excess data is sliently
 * dropped.
 */
void receiveEvent(int bytesReceived) {
  registers[STATUS_REGISTER] = BUSY_STATUS;
  for( byte i = 0; i < bytesReceived; i++ ) {
    if( i < MAX_WRITE_BYTES ) {
      receivedData[i] = Wire.read();
    } else {
      Wire.read();
    }
  }
  bytesReceivedFromMaster = bytesReceived;
}

/**
 * Updates the registers based on what the master wrote to this device. We don't do this in the
 * I2C interrupt handler as it is generally best practice to minimize the amount of time spent
 * in an interrupt handler.
 */
void updateRegisters() {
  if( bytesReceivedFromMaster ) {
    int offset = receivedData[0];
    for(int i = 1; i < bytesReceivedFromMaster; i++ ) {
      int registerIndex = offset + i - 1;
      byte temp = receivedData[i];
      if( registerIndex > 3 && registerIndex < 6 ) { // only registers 4 and 5 are writable
        if( MODE_INDEX == registerIndex ) {
          if( !(temp == CALIBRATION_MODE || temp == RESET_HOME_MODE || temp == TRACKING_MODE) ) {
            temp = TRACKING_MODE;
          }
        }
        registers[registerIndex] = temp;
      }
    }
    bytesReceivedFromMaster = 0;
    registers[STATUS_REGISTER] = isCalibrated? READY_STATUS : UNCALIBRATED_STATUS;
  }
  registers[POSITION_REGISTER] = currentPosition;
  registers[STATE_REGISTER] = encoderState;
}

inline byte isCalibrationActive() {
  updateRegisters(); // check if calibration mode changed via I2C
  return /* LOW == digitalRead(CALIBRATION_PIN) || */ registers[MODE_INDEX] == CALIBRATION_MODE;
}

inline byte isResetHomeActive() {
  updateRegisters();
  return registers[MODE_INDEX] == RESET_HOME_MODE;
}

inline byte toggleResetHome() {
  if( registers[MODE_INDEX] == RESET_HOME_MODE ) {
    registers[MODE_INDEX] = TRACKING_MODE;
  } else {
    registers[MODE_INDEX] = RESET_HOME_MODE;
  }
}

static int eepromReadInt(int addr) {
  byte low = EEPROM.read(addr++);
  byte high = EEPROM.read(addr);
  return word(high, low);
}

static void eepromWriteInt(int addr, int value) {
  byte low = lowByte(value);
  byte high = highByte(value);
  EEPROM.write(addr++, low);
  EEPROM.write(addr, high);
}

static boolean loadSettings() {
  boolean rvalue = true;
  // load settings from EEPROM
  if( 179 == EEPROM.read(0) ) {
    Serial.println("Loading settings");
    // Okay, we've written here before, w00t!
    int k = 1;
    for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
      channels[i].average = eepromReadInt(k);
      k += 2;
      channels[i].minValue = eepromReadInt(k);
      k += 2;
      channels[i].maxValue = eepromReadInt(k);
      k += 2;
    }
    printChannelInfo();
  } else {
    Serial.println("No settings to load");
    rvalue = false;
  }
  return rvalue;
}

static boolean saveSettings() {
  boolean rvalue = true;
  Serial.println("Saving settings");
  EEPROM.write(0, 179);
  int k = 1;
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    eepromWriteInt(k, channels[i].average);
    k += 2;
    eepromWriteInt(k, channels[i].minValue);
    k += 2;
    eepromWriteInt(k, channels[i].maxValue);
    k += 2;
  }
  Serial.println("Done saving settings");
  return rvalue;
}

static void printChannelInfo() {
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    Serial.print("Average: "); Serial.println(channels[i].average);
    Serial.print("Min: "); Serial.println(channels[i].minValue);
    Serial.print("Max: "); Serial.println(channels[i].maxValue);
  }
}

static void updatePotentiometers() {
  Serial.println("Updating potentiometers");
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    pot.write(channels[i].channel, map(channels[i].average, 0, 1023, 0, 255));
    attachInterrupt(channels[i].interrupt, channels[i].isrFunc, CHANGE);
  }
}

void setup() {
  SPI.begin();
  Serial.begin(9600);
  
  channels[0].inputPin = CHANNEL_A_INPUT_PIN;
  channels[0].rawInputPin = CHANNEL_A_RAW_INPUT_PIN;
  channels[0].channel = MCP42xxx::CHANNEL_0;
  channels[0].interrupt = CHANNEL_A_INTERRUPT;
  channels[0].isrFunc = &channelAISR;
  
  channels[1].inputPin = CHANNEL_B_INPUT_PIN;
  channels[1].rawInputPin = CHANNEL_B_RAW_INPUT_PIN;
  channels[1].channel = MCP42xxx::CHANNEL_1;
  channels[1].interrupt = CHANNEL_B_INTERRUPT;
  channels[1].isrFunc = &channelBISR;
  
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    pinMode(channels[i].inputPin, INPUT);
    pinMode(channels[i].rawInputPin, INPUT);
  }
  
  // set up calibration and reset pin
  //pinMode(CALIBRATION_PIN, INPUT);
  pinMode(RESET_PIN, INPUT);
  pinMode(INDEX_INDICATOR_PIN, OUTPUT);
  digitalWrite(INDEX_INDICATOR_PIN, HIGH);
  
  // setup I2C interface
  initializeRegisters();
  
  // load settings from EEPROM
  isCalibrated = loadSettings();
  if( isCalibrated ) {
    isCalibrated = 1;
    registers[STATUS_REGISTER] = READY_STATUS;
    updatePotentiometers();
  } else {
    isCalibrated = 0;
    registers[STATUS_REGISTER] = UNCALIBRATED_STATUS;
  }
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {
  static byte revolutions = 0;
  static byte previousStepIndex = 0;
  static byte previousState = 0;
  static byte previousDirection = CLOCKWISE;
  
  int positionTemp = currentPosition;
  
  // handle calibration
  if( isCalibrationActive() ) {
    Serial.println("Calibrating");
    
    //noInterrupts();
    detachInterrupt(0);
    detachInterrupt(1);
    
    // set default values
    long total[NUMBER_CHANNELS];
    for(int i = 0; i < NUMBER_CHANNELS; i++ ) {
      total[i] = 0;
      channels[i].average = 0;
      channels[i].minValue = 1023;
      channels[i].maxValue = 0;
    }
    
    // collect readings
    long count = 0;
    while( isCalibrationActive() ) {
      for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
        int value = sample(channels[i].rawInputPin);
        channels[i].minValue = min(channels[i].minValue, value);
        channels[i].maxValue = max(channels[i].maxValue, value);
        total[i] += value;
        count++;
      }
    }
    
    // set averages and adjust digital potentiometers accordingly
    for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
      //channels[i].average = total[i] / count;
      channels[i].average = (channels[i].minValue + channels[i].maxValue) / 2;
      printChannelInfo();
    }
    updatePotentiometers();
    saveSettings();
  } // end calibration
  
  // handle reset
  if( isResetHomeActive() ) {
    Serial.println("Reset");
    positionTemp = 0;
    currentPosition = 0;
    revolutions = 0;
    toggleResetHome();
  }
  
  // handle state change
  byte currentState = encoderState;
  if( previousState != currentState ) {
    previousState = currentState;
    
    // determine if direction changed
    byte currentDirection = getDirection(currentState);
    if( currentDirection != previousDirection ) {
      previousDirection = currentDirection;
      Serial.println("Direction changed");
    }
    
    // determine step change
    byte currentStepIndex = (currentState&B00001111) + 1;
    byte delta;
    if( currentStepIndex > previousStepIndex ) {
      delta = currentStepIndex - previousStepIndex;
    } else {
      delta = 16 - previousStepIndex + currentStepIndex;
    }
    
    // update positionTemp
    if( currentDirection == CLOCKWISE ) {
      positionTemp += delta;
    } else {
      positionTemp -= delta;
    }
    
    // determine if we completed a revolution and fix 
    byte revolutionCompleted = 0;
    if( positionTemp >= DIVISIONS ) {
      revolutionCompleted = 1;
      positionTemp -= DIVISIONS;
    } else if( positionTemp <= 0 ) {
      revolutionCompleted = 1;
      positionTemp += DIVISIONS;
    } 

    // determine if a revolution has been completed.
    if( revolutionCompleted ) {
      Serial.print("Index: "); Serial.println(revolutions++);
      digitalWrite(INDEX_INDICATOR_PIN, LOW);
    } else {
      digitalWrite(INDEX_INDICATOR_PIN, HIGH);
    }
    
    // update state
    currentPosition = positionTemp;
    previousStepIndex = currentStepIndex;
    
  } // end handle state change
  
  updateRegisters();
}


