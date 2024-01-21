#define VERSION 'j'

#define INCLUDE_WS2812B
#define INCLUDE_TM1637
#define INCLUDE_GAMEPAD
#define INCLUDE_ENCODERS
#define INCLUDE_BUTTONS
#define INCLUDE_BUTTONMATRIX

#include <avr/pgmspace.h>

#include <EEPROM.h>

#include <SPI.h>

#include "Arduino.h"

#include <avr/pgmspace.h>

#include <Wire.h>

#include "Adafruit_GFX.h"

#include "FlowSerialRead.h"

#include "setPwmFrequency.h"

#include "SHDebouncer.h"

#include "SHButton.h"

#define DEVICE_NAME "SimHub Dash"
#define DEVICE_UNIQUE_ID "183454c8-b199-4b26-ad50-a846561f3136"

#define ENABLE_MICRO_GAMEPAD 1
#define MICRO_GAMEPAD_ENCODERPRESSTIME 50

#define TM1638_ENABLEDMODULES 0
#define TM1638_SWAPLEDCOLORS 0
#define TM1638_DIO 8
#define TM1638_CLK 7
#define TM1638_STB1 9
#define TM1638_SINGLECOLOR1 0
#define TM1638_STB2 10
#define TM1638_SINGLECOLOR2 0
#define TM1638_STB3 11
#define TM1638_SINGLECOLOR3 0
#define TM1638_STB4 12
#define TM1638_SINGLECOLOR4 0
#define TM1638_STB5 0
#define TM1638_SINGLECOLOR5 0
#define TM1638_STB6 0
#define TM1638_SINGLECOLOR6 0

#define TM1637_ENABLEDMODULES 1

#define TM1637_DIO1 4
#define TM1637_CLK1 3
#define TM1637_DIO2 4
#define TM1637_CLK2 3
#define TM1637_DIO3 4
#define TM1637_CLK3 3
#define TM1637_DIO4 4
#define TM1637_CLK4 3
#define TM1637_DIO5 4
#define TM1637_CLK5 3
#define TM1637_DIO6 4
#define TM1637_CLK6 3
#define TM1637_DIO7 4
#define TM1637_CLK7 3
#define TM1637_DIO8 4
#define TM1637_CLK8 3

#include "SHTM1637.h"


#define TM1637_6D_ENABLEDMODULES 0
#define TM1637_6D_DIO1 4
#define TM1637_6D_CLK1 3
#define TM1637_6D_DIO2 4
#define TM1637_6D_CLK2 3
#define TM1637_6D_DIO3 4
#define TM1637_6D_CLK3 3
#define TM1637_6D_DIO4 4
#define TM1637_6D_CLK4 3
#define TM1637_6D_DIO5 4
#define TM1637_6D_CLK5 3
#define TM1637_6D_DIO6 4
#define TM1637_6D_CLK6 3
#define TM1637_6D_DIO7 4
#define TM1637_6D_CLK7 3
#define TM1637_6D_DIO8 4
#define TM1637_6D_CLK8 3

#define MAX7221_ENABLEDMODULES 0
#define MAX7221_DATA 3
#define MAX7221_CLK 5
#define MAX7221_LOAD 4

#define MAX7221_MATRIX_ENABLED 0
#define MAX7221_MATRIX_DATA 3
#define MAX7221_MATRIX_CLK 5
#define MAX7221_MATRIX_LOAD 4

#define ENABLE_ADA_HT16K33_SingleColorMatrix 0
#define ADA_HT16K33_SINGLECOLORMATRIX_I2CADDRESS 0x70

#define WS2812B_RGBLEDCOUNT 2

#define WS2812B_DATAPIN 6
#define WS2812B_RGBENCODING 0
#define WS2812B_RIGHTTOLEFT 0
#define WS2812B_TESTMODE 0
#define WS2812B_USEADAFRUITLIBRARY 0

#if(WS2812B_USEADAFRUITLIBRARY == 0)#include "SHRGBLedsNeoPixelFastLed.h"

SHRGBLedsNeoPixelFastLeds shRGBLedsWS2812B;#include "SHRGBLedsNeoPixel.h"

SHRGBLedsNeoPixel shRGBLedsWS2812B;
Adafruit_NeoPixel WS2812B_strip = Adafruit_NeoPixel(WS2812B_RGBLEDCOUNT, WS2812B_DATAPIN, (WS2812B_RGBENCODING == 0 ? NEO_GRB : (WS2812B_RGBENCODING == 1 ? NEO_RGB : NEO_BRG)) + NEO_KHZ800);

#define PL9823_RGBLEDCOUNT 0
#define PL9823_DATAPIN 6
#define PL9823_RIGHTTOLEFT 0
#define PL9823_TESTMODE 0

#define WS2801_RGBLEDCOUNT 0
#define WS2801_RIGHTTOLEFT 0
#define WS2801_DATAPIN 5
#define WS2801_CLOCKPIN 6
#define WS2801_TESTMODE 0

#define WS2812B_MATRIX_ENABLED 0

#define WS2812B_MATRIX_DATAPIN 6
#define WS2812B_MATRIX_SERPENTINELAYOUT 0

#define DM163_MATRIX_ENABLED 0

#define I2CLCD_enabled 0
#define I2CLCD_size 0
#define I2CLCD_ADDRESS 0x3f
#define I2CLCD_LIBRARY 0
#define I2CLCD_TEST 0
#define I2CLCD_20x4
#define I2CLCD_WIDTH 20
#define I2CLCD_HEIGHT 4
#define I2CLCD_WIDTH 16
#define I2CLCD_HEIGHT 2
#define I2CLCD_PCF8574AT

#define ENABLE_ADA_HT16K33_7SEGMENTS 0

#define ENABLE_ADA_HT16K33_BiColorMatrix 0
#define ADA_HT16K33_BICOLORMATRIX_I2CADDRESS 0x70

#define ENABLE_TACHOMETER 0
#define TACHOMETER_PIN 9

#define ENABLE_SPEEDOGAUGE 0
#define SPEEDO_PIN 4

#define ENABLE_BOOSTGAUGE 0
#define BOOST_PIN 5

#define ENABLE_TEMPGAUGE 0
#define TEMP_PIN 5

#define ENABLE_FUELGAUGE 0
#define FUEL_PIN 5

#define ENABLE_CONSGAUGE 0
#define CONS_PIN 5

#define ENABLED_BUTTONS_COUNT 4

#define BUTTON_PIN_1 5
#define BUTTON_PIN_2 7
#define BUTTON_PIN_3 8
#define BUTTON_PIN_4 9
#define BUTTON_PIN_5 3
#define BUTTON_PIN_6 3
#define BUTTON_PIN_7 3
#define BUTTON_PIN_8 3
#define BUTTON_PIN_9 3
#define BUTTON_PIN_10 3
#define BUTTON_PIN_11 3
#define BUTTON_PIN_12 3

int BUTTON_PINS[] = {
  BUTTON_PIN_1,
  BUTTON_PIN_2,
  BUTTON_PIN_3,
  BUTTON_PIN_4,
  BUTTON_PIN_5,
  BUTTON_PIN_6,
  BUTTON_PIN_7,
  BUTTON_PIN_8,
  BUTTON_PIN_9,
  BUTTON_PIN_10,
  BUTTON_PIN_11,
  BUTTON_PIN_12
};
SHButton button1, button2, button3, button4, button5, button6, button7, button8, button9, button10, button11, button12;
SHButton * BUTTONS[] = {
  & button1,
  & button2,
  & button3,
  & button4,
  & button5,
  & button6,
  & button7,
  & button8,
  & button9,
  & button10,
  & button11,
  & button12
};

SHDebouncer ButtonsDebouncer(10);

#define ENABLED_ENCODERS_COUNT 1#include "SHRotaryEncoder.h"

#define ENCODER1_CLK_PIN 10
#define ENCODER1_DT_PIN 11
#define ENCODER1_BUTTON_PIN 12
#define ENCODER1_ENABLE_PULLUP 0
#define ENCODER1_REVERSE_DIRECTION 0
#define ENCODER1_ENABLE_HALFSTEPS 0

#define ENCODER2_CLK_PIN 11
#define ENCODER2_DT_PIN 12
#define ENCODER2_BUTTON_PIN 13
#define ENCODER2_ENABLE_PULLUP 0
#define ENCODER2_REVERSE_DIRECTION 0
#define ENCODER2_ENABLE_HALFSTEPS 0

#define ENCODER3_CLK_PIN 7
#define ENCODER3_DT_PIN 8
#define ENCODER3_BUTTON_PIN 9
#define ENCODER3_ENABLE_PULLUP 0
#define ENCODER3_REVERSE_DIRECTION 0
#define ENCODER3_ENABLE_HALFSTEPS 0

#define ENCODER4_CLK_PIN 7
#define ENCODER4_DT_PIN 8
#define ENCODER4_BUTTON_PIN 9
#define ENCODER4_ENABLE_PULLUP 0
#define ENCODER4_REVERSE_DIRECTION 0
#define ENCODER4_ENABLE_HALFSTEPS 0

#define ENCODER5_CLK_PIN 7
#define ENCODER5_DT_PIN 8
#define ENCODER5_BUTTON_PIN 9
#define ENCODER5_ENABLE_PULLUP 0
#define ENCODER5_REVERSE_DIRECTION 0
#define ENCODER5_ENABLE_HALFSTEPS 0

#define ENCODER6_CLK_PIN 7
#define ENCODER6_DT_PIN 8
#define ENCODER6_BUTTON_PIN 9
#define ENCODER6_ENABLE_PULLUP 0
#define ENCODER6_REVERSE_DIRECTION 0
#define ENCODER6_ENABLE_HALFSTEPS 0

#define ENCODER7_CLK_PIN 7
#define ENCODER7_DT_PIN 8
#define ENCODER7_BUTTON_PIN 9
#define ENCODER7_ENABLE_PULLUP 0
#define ENCODER7_REVERSE_DIRECTION 0
#define ENCODER7_ENABLE_HALFSTEPS 0

#define ENCODER8_CLK_PIN 7
#define ENCODER8_DT_PIN 8
#define ENCODER8_BUTTON_PIN 9
#define ENCODER8_ENABLE_PULLUP 0
#define ENCODER8_REVERSE_DIRECTION 0
#define ENCODER8_ENABLE_HALFSTEPS 0

SHRotaryEncoder encoder1, encoder2, encoder3, encoder4, encoder5, encoder6, encoder7, encoder8;
SHRotaryEncoder * SHRotaryEncoders[] = {
  & encoder1,
  & encoder2,
  & encoder3,
  & encoder4,
  & encoder5,
  & encoder6,
  & encoder7,
  & encoder8
};

#define ENABLED_BUTTONMATRIX 1

#define BMATRIX_COLS 3
#define BMATRIX_ROWS 3

#include "SHButtonMatrix.h"

#define BMATRIX_COL1 2
#define BMATRIX_COL2 13
#define BMATRIX_COL3 14
#define BMATRIX_COL4 5
#define BMATRIX_COL5 2
#define BMATRIX_COL6 2
#define BMATRIX_COL7 2
#define BMATRIX_COL8 2

#define BMATRIX_ROW1 15
#define BMATRIX_ROW2 16
#define BMATRIX_ROW3 17
#define BMATRIX_ROW4 9
#define BMATRIX_ROW5 2
#define BMATRIX_ROW6 2
#define BMATRIX_ROW7 2
#define BMATRIX_ROW8 2

byte BMATRIX_COLSDEF[8] = {
  BMATRIX_COL1,
  BMATRIX_COL2,
  BMATRIX_COL3,
  BMATRIX_COL4,
  BMATRIX_COL5,
  BMATRIX_COL6,
  BMATRIX_COL7,
  BMATRIX_COL8
};
byte BMATRIX_ROWSDEF[8] = {
  BMATRIX_ROW1,
  BMATRIX_ROW2,
  BMATRIX_ROW3,
  BMATRIX_ROW4,
  BMATRIX_ROW5,
  BMATRIX_ROW6,
  BMATRIX_ROW7,
  BMATRIX_ROW8
};

SHButtonMatrix shButtonMatrix;

#define ADAMOTORS_SHIELDSCOUNT 0
#define ADAMOTORS_FREQ 1900

#define MOTOMONSTER_ENABLED 0
#define MOTOMONSTER_REVERSEDIRECTION 0

#define DKMOTOR_SHIELDSCOUNT 0
#define DKMOTOR_USEHUMMINGREDUCING 0

#define L98NMOTORS_ENABLED 0
#define L98N_enA 10
#define L98N_in1 9
#define L98N_in2 8
#define L98N_enB 5
#define L98N_in3 7
#define L98N_in4 6

#define SHAKEITPWM_ENABLED_MOTORS 0
#define SHAKEITPWM_O1 5
#define SHAKEITPWM_MIN_OUTPUT_O1 0
#define SHAKEITPWM_MAX_OUTPUT_O1 255
#define SHAKEITPWM_O2 6
#define SHAKEITPWM_MIN_OUTPUT_O2 0
#define SHAKEITPWM_MAX_OUTPUT_O2 255
#define SHAKEITPWM_O3 9
#define SHAKEITPWM_MIN_OUTPUT_O3 0
#define SHAKEITPWM_MAX_OUTPUT_O3 255
#define SHAKEITPWM_O4 10
#define SHAKEITPWM_MIN_OUTPUT_O4 0
#define SHAKEITPWM_MAX_OUTPUT_O4 255

#define SHAKEITPWMFANS_ENABLED_MOTORS 0

#define SHAKEITPWMFANS_O1 9
#define SHAKEITPWMFANS_MIN_OUTPUT_O1 0
#define SHAKEITPWMFANS_MAX_OUTPUT_O1 255
#define SHAKEITPWMFANS_RELAY_PIN_01 4
#define SHAKEITPWMFANS_RELAY_DELAY_01 2000
#define SHAKEITPWMFANS_O2 10
#define SHAKEITPWMFANS_MIN_OUTPUT_O2 0
#define SHAKEITPWMFANS_MAX_OUTPUT_O2 255
#define SHAKEITPWMFANS_RELAY_PIN_02 5
#define SHAKEITPWMFANS_RELAY_DELAY_02 2000
#define SHAKEITPWMFANS_O3 11
#define SHAKEITPWMFANS_MIN_OUTPUT_O3 0
#define SHAKEITPWMFANS_MAX_OUTPUT_O3 255
#define SHAKEITPWMFANS_RELAY_PIN_03 6
#define SHAKEITPWMFANS_RELAY_DELAY_03 2000
#define SHAKEITPWMFANS_O4 10
#define SHAKEITPWMFANS_MIN_OUTPUT_O4 0
#define SHAKEITPWMFANS_MAX_OUTPUT_O4 255
#define SHAKEITPWMFANS_RELAY_PIN_04 7
#define SHAKEITPWMFANS_RELAY_DELAY_04 2000

#define ENABLED_OLEDLCD 0
#define ENABLED_NOKIALCD 0

#define ENABLED_NOKIALCD 0

#define ENABLED_OLEDLCD 0

#define ENABLE_74HC595_GEAR_DISPLAY 0
#define RS_74HC595_DATAPIN 2
#define RS_74HC595_LATCHPIN 3
#define RS_74HC595_CLOCKPIN 4

#define ENABLE_6C595_GEAR_DISPLAY 0
#define RS_6c595_DATAPIN 11
#define RS_6c595_LATCHPIN 13
#define RS_6c595_SLAVEPIN 10

#include "SHLedsBackpack.h"


#include <Joystick.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, 128, 0,
  false, false, false, false, false, false,
  false, false, false, false, false);

#include "SHCustomProtocol.h"

SHCustomProtocol shCustomProtocol;#include "SHCommands.h"

#include "SHCommandsGlcd.h"

unsigned long lastMatrixRefresh = 0;
void idle(bool critical) {
  for (int i = 0; i < ENABLED_ENCODERS_COUNT; i++) {
    SHRotaryEncoders[i] -> read();
  }

  shButtonMatrix.read();

  if (ButtonsDebouncer.Debounce()) {
    bool changed = false;
    for (int btnIdx = 0; btnIdx < ENABLED_BUTTONS_COUNT; btnIdx++) {
      BUTTONS[btnIdx] -> read();
    }
    if (changed) {
      UpdateGamepadState();
    }
    shCustomProtocol.idle();
  }
}

void EncoderPositionChanged(int encoderId, int position, byte direction) {
  UpdateGamepadEncodersState(true);
  if (direction < 2) {
    arqserial.CustomPacketStart(0x01, 3);
    arqserial.CustomPacketSendByte(encoderId);
    arqserial.CustomPacketSendByte(direction);
    arqserial.CustomPacketSendByte(position);
    arqserial.CustomPacketEnd();
  } else {
    arqserial.CustomPacketStart(0x02, 2);
    arqserial.CustomPacketSendByte(encoderId);
    arqserial.CustomPacketSendByte(direction - 2);
    arqserial.CustomPacketEnd();
  }
}

void buttonStatusChanged(int buttonId, byte Status) {
  Joystick.setButton(TM1638_ENABLEDMODULES * 8 + buttonId - 1, Status);
  Joystick.sendState();
  arqserial.CustomPacketStart(0x03, 2);
  arqserial.CustomPacketSendByte(buttonId);
  arqserial.CustomPacketSendByte(Status);
  arqserial.CustomPacketEnd();
}

void buttonMatrixStatusChanged(int buttonId, byte Status) {
  Joystick.setButton(TM1638_ENABLEDMODULES * 8 + ENABLED_BUTTONS_COUNT + buttonId - 1, Status);
  Joystick.sendState();
  arqserial.CustomPacketStart(0x03, 2);
  arqserial.CustomPacketSendByte(ENABLED_BUTTONS_COUNT + buttonId);
  arqserial.CustomPacketSendByte(Status);
  arqserial.CustomPacketEnd();
}

void setup() {

  FlowSerialBegin(19200);

  Joystick.begin(false);

  TM1637_Init();

  #if(WS2812B_USEADAFRUITLIBRARY == 0)#include "SHRGBLedsNeoPixelFastLed.h"

  shRGBLedsWS2812B.begin(WS2812B_RGBLEDCOUNT, WS2812B_RIGHTTOLEFT, WS2812B_TESTMODE);#include "SHRGBLedsNeoPixel.h"

  shRGBLedsWS2812B.begin( & WS2812B_strip, WS2812B_RGBLEDCOUNT, WS2812B_RIGHTTOLEFT, WS2812B_TESTMODE);

  for (int btnIdx = 0; btnIdx < ENABLED_BUTTONS_COUNT; btnIdx++) {
    BUTTONS[btnIdx] -> begin(btnIdx + 1, BUTTON_PINS[btnIdx], buttonStatusChanged);
  }

  shButtonMatrix.begin(BMATRIX_COLS, BMATRIX_ROWS, BMATRIX_COLSDEF, BMATRIX_ROWSDEF, buttonMatrixStatusChanged);

  InitEncoders();

  shCustomProtocol.setup();
  arqserial.setIdleFunction(idle);
}

void InitEncoders() {
  if (ENABLED_ENCODERS_COUNT > 0) encoder1.begin(ENCODER1_CLK_PIN, ENCODER1_DT_PIN, ENCODER1_BUTTON_PIN, ENCODER1_REVERSE_DIRECTION, ENCODER1_ENABLE_PULLUP, 1, ENCODER1_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 1) encoder2.begin(ENCODER2_CLK_PIN, ENCODER2_DT_PIN, ENCODER2_BUTTON_PIN, ENCODER2_REVERSE_DIRECTION, ENCODER2_ENABLE_PULLUP, 2, ENCODER2_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 2) encoder3.begin(ENCODER3_CLK_PIN, ENCODER3_DT_PIN, ENCODER3_BUTTON_PIN, ENCODER3_REVERSE_DIRECTION, ENCODER3_ENABLE_PULLUP, 3, ENCODER3_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 3) encoder4.begin(ENCODER4_CLK_PIN, ENCODER4_DT_PIN, ENCODER4_BUTTON_PIN, ENCODER4_REVERSE_DIRECTION, ENCODER4_ENABLE_PULLUP, 4, ENCODER4_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 4) encoder5.begin(ENCODER5_CLK_PIN, ENCODER5_DT_PIN, ENCODER5_BUTTON_PIN, ENCODER5_REVERSE_DIRECTION, ENCODER5_ENABLE_PULLUP, 5, ENCODER5_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 5) encoder6.begin(ENCODER6_CLK_PIN, ENCODER6_DT_PIN, ENCODER6_BUTTON_PIN, ENCODER6_REVERSE_DIRECTION, ENCODER6_ENABLE_PULLUP, 6, ENCODER6_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 6) encoder7.begin(ENCODER7_CLK_PIN, ENCODER7_DT_PIN, ENCODER7_BUTTON_PIN, ENCODER7_REVERSE_DIRECTION, ENCODER7_ENABLE_PULLUP, 7, ENCODER7_ENABLE_HALFSTEPS, EncoderPositionChanged);
  if (ENABLED_ENCODERS_COUNT > 7) encoder8.begin(ENCODER8_CLK_PIN, ENCODER8_DT_PIN, ENCODER8_BUTTON_PIN, ENCODER8_REVERSE_DIRECTION, ENCODER8_ENABLE_PULLUP, 8, ENCODER8_ENABLE_HALFSTEPS, EncoderPositionChanged);
}

void UpdateGamepadState() {
  int btnidx = 0;

  UpdateGamepadEncodersState(false);

  Joystick.sendState();
}

void UpdateGamepadEncodersState(bool sendState) {
  int btnidx = TM1638_ENABLEDMODULES * 8 + ENABLED_BUTTONS_COUNT + ENABLED_BUTTONMATRIX * (BMATRIX_COLS * BMATRIX_ROWS);
  unsigned long refTime = millis();
  for (int i = 0; i < ENABLED_ENCODERS_COUNT; i++) {
    uint8_t dir = SHRotaryEncoders[i] -> getDirection(MICRO_GAMEPAD_ENCODERPRESSTIME, refTime);
    Joystick.setButton(btnidx, dir == 0);
    Joystick.setButton(btnidx + 1, dir == 1);
    Joystick.setButton(btnidx + 2, SHRotaryEncoders[i] -> getPressed());

    btnidx += 3;
  }

  if (sendState)
    Joystick.sendState();
}

char loop_opt;
unsigned long lastSerialActivity = 0;

void loop() {
  UpdateGamepadState();

  shCustomProtocol.loop();

  if (FlowSerialAvailable() > 0) {
    if (FlowSerialTimedRead() == MESSAGE_HEADER) {
      lastSerialActivity = millis();

      loop_opt = FlowSerialTimedRead();

      if (loop_opt == '1') Command_Hello();
      else if (loop_opt == '8') Command_SetBaudrate();
      else if (loop_opt == 'J') Command_ButtonsCount();
      else if (loop_opt == '2') Command_TM1638Count();
      else if (loop_opt == 'B') Command_SimpleModulesCount();
      else if (loop_opt == 'A') Command_Acq();
      else if (loop_opt == 'N') Command_DeviceName();
      else if (loop_opt == 'I') Command_UniqueId();
      else if (loop_opt == '0') Command_Features();
      else if (loop_opt == '3') Command_TM1638Data();
      else if (loop_opt == 'V') Command_Motors();
      else if (loop_opt == 'S') Command_7SegmentsData();
      else if (loop_opt == '4') Command_RGBLEDSCount();
      else if (loop_opt == '6') Command_RGBLEDSData();
      else if (loop_opt == 'R') Command_RGBMatrixData();
      else if (loop_opt == 'M') Command_MatrixData();
      else if (loop_opt == 'G') Command_GearData();
      else if (loop_opt == 'L') Command_I2CLCDData();
      else if (loop_opt == 'K') Command_GLCDData();
      else if (loop_opt == 'P') Command_CustomProtocolData();
      else if (loop_opt == 'X') {
        String xaction = FlowSerialReadStringUntil(' ', '\n');
        if (xaction == F("list")) Command_ExpandedCommandsList();
        else if (xaction == F("mcutype")) Command_MCUType();
        else if (xaction == F("tach")) Command_TachData();
        else if (xaction == F("speedo")) Command_SpeedoData();
        else if (xaction == F("boost")) Command_BoostData();
        else if (xaction == F("temp")) Command_TempData();
        else if (xaction == F("fuel")) Command_FuelData();
        else if (xaction == F("cons")) Command_ConsData();
        else if (xaction == F("encoderscount")) Command_EncodersCount();
      }
    }
  }

  if (millis() - lastSerialActivity > 5000) {
    Command_Shutdown();
  }
}