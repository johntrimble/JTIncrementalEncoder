#include<SPI.h>
#include "encoder.h"
#include<MCP42xxx.h>

MCP42xxx pot(10, -1, -1);

// General encoder constants
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

// Interrupt vars
static byte isrSteps = 0;

// Current encoder reading, format: 
// - First 4 bits are a counter incremented by one each time the state is updated. 
// - 5th bit is the state of channel A
// - 6th bit is the state of channel B
// - 7th bit is the previous state of channel A
// - 8th bit is the previous state of channel B
static volatile byte encoderState = 0;

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
  for( int i = 0; i < length; i++ ) {
    int value = arr[i];
    for( int k = i; k > 0 && arr[k-1] > arr[k]; k-- ) {
      int tmp = arr[k];
       arr[k] = arr[k-1];
       arr[k-1] = tmp;
       k--;
    }
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
  pinMode(CALIBRATION_PIN, INPUT);
  pinMode(RESET_PIN, INPUT);
  pinMode(INDEX_INDICATOR_PIN, OUTPUT);
  digitalWrite(INDEX_INDICATOR_PIN, HIGH);
}

void loop() {
  static int position = 0;
  static byte revolutions = 0;
  static byte previousStepIndex = 0;
  static byte previousState = 0;
  static byte previousDirection = CLOCKWISE;
  
  // handle calibration
  if( LOW == digitalRead(CALIBRATION_PIN) ) {
    Serial.println("Calibrating");
    
    noInterrupts();
    
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
    while( LOW == digitalRead(CALIBRATION_PIN) ) {
      for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
        int value = sample(channels[i].rawInputPin);
        channels[i].minValue = min(channels[i].minValue, value);
        channels[i].maxValue = max(channels[i].maxValue, value);
        total[i] += value;
        count++;
      }
    }

    // attach interrupts
    for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
      attachInterrupt(channels[i].interrupt, channels[i].isrFunc, CHANGE);
    }
    
    interrupts();
    
    // set averages and adjust digital potentiometers accordingly
    for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
      channels[i].average = total[i] / count;
      pot.write(channels[i].channel, map(channels[i].average, 0, 1023, 0, 255));
      
      Serial.print("Average: "); Serial.println(channels[i].average);
      Serial.print("Min: "); Serial.println(channels[i].minValue);
      Serial.print("Max: "); Serial.println(channels[i].maxValue);
    }
  }
  
  // handle reset
  if( LOW == digitalRead(RESET_PIN) ) {
    Serial.println("Reset");
    position = 0;
    revolutions = 0;
    while( LOW == digitalRead(RESET_PIN) );
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
    previousStepIndex = currentStepIndex;
    
    // update position
    if( currentDirection == CLOCKWISE ) {
      position += delta;
    } else {
      position -= delta;
    }
    
    // determine if we completed a revolution and fix 
    byte revolutionCompleted = 0;
    if( position >= DIVISIONS ) {
      revolutionCompleted = 1;
      position -= DIVISIONS;
    } else if( position <= 0 ) {
      revolutionCompleted = 1;
      position += DIVISIONS;
    } 

    // determine if a revolution has been completed.
    if( revolutionCompleted ) {
      Serial.print("Index: "); Serial.println(revolutions++);
      digitalWrite(INDEX_INDICATOR_PIN, LOW);
    } else {
      digitalWrite(INDEX_INDICATOR_PIN, HIGH);
    }
  }
}


