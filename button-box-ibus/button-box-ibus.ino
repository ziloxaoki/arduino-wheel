#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>
#include "Arduino.h"
#include <avr/pgmspace.h>

bool isInterruptDisabled[] = {false, false, false, false};
const int encoderPinA[] = {21, 20, 19, 18};
const int encoderPinB[] = {25, 24, 27, 26};

// ----------------------- ADDITIONAL BUTTONS ---------------------------------------------------------------
// https://www.arduino.cc/en/Tutorial/InputPullupSerial
// ----------------------------------------------------------------------------------------------------------
int ENABLED_BUTTONS_COUNT = 2;
int BUTTON_PIN_1 = 22;
int BUTTON_PIN_2 = 23;
int BUTTON_PINS[] = { BUTTON_PIN_1, BUTTON_PIN_2 };

int ENABLED_MATRIX_COLUMNS = 7;
int ENABLED_MATRIX_ROWS = 7;
int lastButtonState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int currentButtonState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long lastButtonDebounce[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int rowPins[] = {35, 34, 33, 32, 31, 30, 29};
int columnPins[] = {46, 45, 44, 43, 42, 41, 40};


const int TOTAL_ROTARY = 4;
volatile int encoderPos[] = {15000, 15000, 15000, 15000};
volatile int orientation[] = {0, 0, 0, 0};
int lastRotaryPos[] = {15000, 15000, 15000, 15000};
byte lastRotaryState[] = {0, 0, 0, 0};
long lastRotaryStateChange = 0;
long lastHandshakeSent = 0;
long lastButtonStateSent = 0;

#include "ibus.h"

// How often to send data?
#define UPDATE_INTERVAL 10UL // milliseconds

// Define the appropriate analog reference source. See
// https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
// Based on your device voltage, you may need to modify this definition
#define ANALOG_REFERENCE DEFAULT

// Define the baud rate
#define BAUD_RATE 115200

byte digitalBitmappedPins[] = {};



#define DIGITAL_BITMAPPED_INPUTS_COUNT TOTAL_ROTARY + ENABLED_BUTTONS_COUNT + (ENABLED_MATRIX_COLUMNS * ENABLED_MATRIX_ROWS)
//#define DIGITAL_BITMAPPED_INPUTS_COUNT 64
#define NUM_CHANNELS ((15 + (DIGITAL_BITMAPPED_INPUTS_COUNT))/16)
// #define NUM_CHANNELS 4

IBus ibus(NUM_CHANNELS);


void rotEncoder1() {
  int pinOffset = 0;
  detachInterrupt(digitalPinToInterrupt(encoderPinA[pinOffset]));
  isInterruptDisabled[pinOffset] = true;
  //  if(millis() - lastRotaryBounce > 10) {
  int pinB = digitalRead(encoderPinB[pinOffset]);
  if (pinB == HIGH) {
    encoderPos[pinOffset]++;
  } else {
    encoderPos[pinOffset]--;
  }
  //  }
  //  lastRotaryBounce = millis();
}

void rotEncoder2() {
  int pinOffset = 1;
  detachInterrupt(digitalPinToInterrupt(encoderPinA[pinOffset]));
  isInterruptDisabled[pinOffset] = true;
  //  if(millis() - lastRotaryBounce > 10) {
  int pinB = digitalRead(encoderPinB[pinOffset]);
  if (pinB == HIGH) {
    encoderPos[pinOffset]++;
  } else {
    encoderPos[pinOffset]--;
  }
  //  }
  //  lastRotaryBounce = millis();
}

void rotEncoder3() {
  int pinOffset = 2;
  detachInterrupt(digitalPinToInterrupt(encoderPinA[pinOffset]));
  isInterruptDisabled[pinOffset] = true;
  //  if(millis() - lastRotaryBounce > 10) {
  int pinB = digitalRead(encoderPinB[pinOffset]);
  if (pinB == HIGH) {
    encoderPos[pinOffset]++;
  } else {
    encoderPos[pinOffset]--;
  }
  //  }
  //  lastRotaryBounce = millis();
}

void rotEncoder4() {
  int pinOffset = 3;
  detachInterrupt(digitalPinToInterrupt(encoderPinA[pinOffset]));
  isInterruptDisabled[pinOffset] = true;
  //  if(millis() - lastRotaryBounce > 10) {
  int pinB = digitalRead(encoderPinB[pinOffset]);
  if (pinB == HIGH) {
    encoderPos[pinOffset]++;
  } else {
    encoderPos[pinOffset]--;
  }
  //  }
  //  lastRotaryBounce = millis();
}

void deAttachInterrupts() {
  int i = 0;
  for (i=0; i < 4; i++) {
    if (isInterruptDisabled[i] == false && digitalRead(encoderPinA[i]) != LOW) {
      detachInterrupt(digitalPinToInterrupt(encoderPinA[i]));
      isInterruptDisabled[i] = true;
    }
  }  
}

void reAttachInterrupts() {
  if (isInterruptDisabled[0] == true && digitalRead(encoderPinA[0]) != LOW) {
    attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), rotEncoder1, LOW);
    isInterruptDisabled[0] = false;
  }

  if (isInterruptDisabled[1] == true && digitalRead(encoderPinA[1]) != LOW) {
    attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), rotEncoder2, LOW);
    isInterruptDisabled[1] = false;
  }

  if (isInterruptDisabled[2] == true && digitalRead(encoderPinA[2]) != LOW) {
    attachInterrupt(digitalPinToInterrupt(encoderPinA[2]), rotEncoder3, LOW);
    isInterruptDisabled[2] = false;
  }

  if (isInterruptDisabled[3] == true && digitalRead(encoderPinA[3]) != LOW) {
    attachInterrupt(digitalPinToInterrupt(encoderPinA[3]), rotEncoder4, LOW);
    isInterruptDisabled[3] = false;
  }
}


int sendRotaryState(int offset, byte *response) {
  for (int i = 0; i < TOTAL_ROTARY; i++) {

    if (millis() - lastRotaryStateChange > 50) {
      if (encoderPos[i] != lastRotaryPos[i]) {
        if (lastRotaryPos[i] < encoderPos[i]) {
          //turn left
          lastRotaryState[i] = 1;
        } else {
          //turn right
          lastRotaryState[i] = 2;
        }
        lastRotaryPos[i] = encoderPos[i];

        lastRotaryStateChange = millis();
      } else {
        //not pressed
        lastRotaryState[i] = 0;
      }
    }

    switch (lastRotaryState[i]) {
      case 0:
        response[offset++] = 0;
        response[offset++] = 0;
        break;
      case 1:
        response[offset++] = 0;
        response[offset++] = 1;
        break;
      case 2:
        response[offset++] = 1;
        response[offset++] = 0;
        break;
    }
  }

  return offset;
}


int sendButtonState(int offset, byte *response) {
  for (int i = 0; i < ENABLED_BUTTONS_COUNT; i++) {
    response[offset++] = digitalRead(BUTTON_PINS[i]) == HIGH ? 0 : 1;
  }

  return offset;
}


int sendMatrixState(int offset, byte *response) {
  int aux = 0;

  for (int i = 0; i < ENABLED_MATRIX_COLUMNS; i++) {

    digitalWrite(columnPins[i], LOW);
    for (int x = 0; x < ENABLED_MATRIX_ROWS; x++) {
      byte pinState = (digitalRead(rowPins[x]) == LOW) ? 1 : 0;

      if (lastButtonState[aux] != pinState) {
        lastButtonDebounce[aux] = millis();
        lastButtonState[aux] = pinState;
      }

      if (lastButtonDebounce[aux] > 0 && millis() - lastButtonDebounce[aux]  > 15) {
        currentButtonState[aux] = lastButtonState[aux];
        lastButtonDebounce[aux] = 0;
      }

      response[offset++] = currentButtonState[aux++];

    }

    digitalWrite(columnPins[i], HIGH);
    //delay(10);
  }

  return offset;
}

void initBuffer(byte *buffer, int size) {
  for (int x = 0; x < size; x++) {
    buffer[x] = 0;
  }

  return buffer;
}

void writeToBus(bool isDebug) {
  int i, offset = 0;
  uint16_t bm_ch = 0;         // MUST initialize, MUST be 16-bit

  byte response[DIGITAL_BITMAPPED_INPUTS_COUNT];
  initBuffer(response, DIGITAL_BITMAPPED_INPUTS_COUNT);

  //Serial.println("pqp");
  offset = sendButtonState(offset, response);
  offset = sendRotaryState(offset, response);
  offset = sendMatrixState(offset, response);

  ibus.begin();

  // read digital bit-mapped pins - 16 pins go in one channel
  for(i=0; i < DIGITAL_BITMAPPED_INPUTS_COUNT; i++) {
     int bit = i%16;
     if(response[i] == 1)
        bm_ch |= 1 << bit;

     if(bit == 15 || i == DIGITAL_BITMAPPED_INPUTS_COUNT-1) {
        // data for one channel ready
        if (isDebug) {
          Serial.println(bm_ch);
        } else {
          ibus.write(bm_ch);
        }
        bm_ch = 0;
     }
  }

  ibus.end();
}

void setup()
{
  // EXTERNAL BUTTONS INIT
  for (int btnIdx = 0; btnIdx < ENABLED_BUTTONS_COUNT; btnIdx++) {
    pinMode(BUTTON_PINS[btnIdx], INPUT_PULLUP);
  }

  // MATRIX
  for (int x = 0; x < ENABLED_MATRIX_COLUMNS; x++) {
    pinMode(columnPins[x], OUTPUT);           // set pin to input
    digitalWrite(columnPins[x], HIGH);        // initiate high
    //pinAsOutput(columnPins[x]);
  }

  for (int x = 0; x < ENABLED_MATRIX_ROWS; x++) {
    pinMode(rowPins[x], INPUT_PULLUP);
    //pinAsInputPullUp(rowPins[x]);
  }

  // Rotary
  for (int i = 0; i < TOTAL_ROTARY; i++) {
    pinMode(encoderPinA[i], INPUT_PULLUP);
    pinMode(encoderPinB[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), rotEncoder1, LOW);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), rotEncoder2, LOW);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[2]), rotEncoder3, LOW);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[3]), rotEncoder4, LOW);

  analogReference(ANALOG_REFERENCE); // use the defined ADC reference voltage source
  Serial.begin(BAUD_RATE);           // setup serial
}

void loop() {
  unsigned long start = millis();

  writeToBus(false);

  unsigned long elapsed =  millis() - start; // time elapsed in reading the inputs
  if(elapsed < UPDATE_INTERVAL) {
     delay(UPDATE_INTERVAL - elapsed);
  }

  reAttachInterrupts();
}
