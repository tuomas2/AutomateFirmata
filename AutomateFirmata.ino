//#define LOWPOWER
//#define VIRTUALWIRE
//#define LIQUIDCRYSTAL

#include <Firmata.h>
#include <EEPROM.h>
#include <Wire.h>

#ifdef LOWPOWER
   #include <LowPower.h>
#endif

#ifdef VIRTUALWIRE
  #include <VirtualWire.h>
#endif

#ifdef LIQUIDCRYSTAL
  #include <LiquidCrystal_I2C.h>
#endif

/*

   AutomateFirmata. Firmata with some additional features for Automate 
   (https://github.com/tuomas2/automate)

   Repository URL: https://github.com/tuomas2/AutomateFirmata

   Based on StandardFirmata. 

   Firmata is a generic protocol for communicating with microcontrollers
   from software on a host computer. It is intended to work with
   any host computer software package.

   To download a host software package, please click on the following link
   to open the list of Firmata client libraries in your default browser.

   https://github.com/firmata/arduino#firmata-client-libraries
   
   Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
   Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
   Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
   Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.
   Copyright (C) 2017 Tuomas Airaksinen. All rights reserved.
   
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   
   See file LICENSE.txt for further informations on licensing terms.
   
   TODO
   
   - Test high frequency PWM 
   - Test power saving (measurements)
   - Power saving by disabling AD converter if ports not used
   - Make power-saving configurable
   
 */



#define DEBUG 0

#if DEBUG
  char tmpBuf[64];
  #define dbgf(f_, ...) snprintf_P(tmpBuf, sizeof(tmpBuf), PSTR(f_), __VA_ARGS__); Firmata.sendString(tmpBuf);
  #define dbg(f_) snprintf_P(tmpBuf, sizeof(tmpBuf), PSTR(f_)); Firmata.sendString(tmpBuf);
#else
  #define dbgf(...);
  #define dbg(f_);
#endif

#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1
#define DEFAULT_SAMPLING_INTERVAL 500
#define DEFAULT_VIRTUALWIRE_SPEED 2
#define BLINK_INTERVAL 0
#define BLINK_PIN 13
#define SERIAL_SHUTDOWN_TIME 120000 // 2 minutes

// VirtualWire.h does not export this but we need it.
extern "C"
{
  extern void vw_tx_stop();
}

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/


#ifdef LIQUIDCRYSTAL
LiquidCrystal_I2C *lcd = NULL;
LiquidCrystal_I2C lcd0(0x27, 16, 2);
#else
int lcd = 0;
#endif


char lcdBuf[20];  

byte lcdPort = 0x27;
byte lcdColumns = 16;
byte lcdRows = 2;

byte lcdReporting = false;
unsigned short int reportPin = 0; // currently being reported
unsigned long previousLCDMillis = 0;

uint8_t homeId = 0x01; // Set this different to your neighbors 
uint8_t deviceId = 0x01; // Set this individual within your home

uint8_t vwPttPin = 0;
uint8_t vwRxPin = 0; // 0 disabled, otherwise pin number
uint8_t vwTxPin = 0;
uint8_t wakeUpPin = 0; // 2 or 3
uint8_t virtualWireSpeed = DEFAULT_VIRTUALWIRE_SPEED; // * 1000 bits per second 

boolean serialEnabled = true;
boolean instantDigitalReporting = true;
unsigned long digitalOutputMillis = 0;

#ifdef LOWPOWER
period_t sleepMode = SLEEP_1S;
#endif 
int sleepTime = 1000;

static const int BROADCAST_RECIPIENT = 0xFF;
static const uint8_t HEADER_LENGTH = 4;

// Incoming sysexs (0x00-0x0F are user defined according to FirmataConstants.h, let's use those)
static const byte SYSEX_VIRTUALWIRE_MESSAGE = 0x00; // This is used both incoming and outgoing
static const byte SYSEX_KEEP_ALIVE = 0x01;
static const byte SYSEX_SETUP_VIRTUALWIRE = 0x02;
static const byte SYSEX_SETUP_LCD = 0x03;
static const byte SYSEX_LCD_COMMAND = 0x04;
static const byte SYSEX_SET_ANALOG_REFERENCE = 0x05;
static const byte SYSEX_SET_INSTANT_DIGITAL_REPORTING = 0x06;

// LCD command bytes
static const byte LCD_SET_BACKLIGHT = 0x01;
static const byte LCD_PRINT = 0x02;
static const byte LCD_CLEAR = 0x03;
static const byte LCD_SET_CURSOR = 0x04;
static const byte LCD_SET_REPORTING = 0x05;

// Virtualwire command bytes
static const byte VIRTUALWIRE_SET_PIN_MODE = 0x01;
static const byte VIRTUALWIRE_ANALOG_MESSAGE = 0x02;
static const byte VIRTUALWIRE_DIGITAL_MESSAGE = 0x03;
static const byte VIRTUALWIRE_START_SYSEX = 0x04;
static const byte VIRTUALWIRE_SET_DIGITAL_PIN_VALUE = 0x05;
static const byte VIRTUALWIRE_DIGITAL_BROADCAST = 0x06;
static const byte VIRTUALWIRE_ANALOG_BROADCAST = 0x07;

// EEPROM addressses
static const int EEPROM_HOME_ID = 0;
static const int EEPROM_DEVICE_ID = 1;
static const int EEPROM_VIRTUALWIRE_RX_PIN = 2;
static const int EEPROM_VIRTUALWIRE_TX_PIN = 3;
static const int EEPROM_VIRTUALWIRE_PTT_PIN = 4;
static const int EEPROM_VIRTUALWIRE_SPEED = 5;
static const int EEPROM_WAKEUP_PIN = 6;
static const int EEPROM_SAMPLING_INTERVAL = 7; // 2 bytes 
static const int EEPROM_ANALOG_INPUTS_TO_REPORT = 9; // 2 byte
static const int EEPROM_LCD_PORT = 11;
static const int EEPROM_LCD_COLUMNS = 12;
static const int EEPROM_LCD_ROWS = 13;
static const int EEPROM_CONFIGURED = 14;
static const int EEPROM_CONFIG_VERSION = 15;
static const int EEPROM_LCD_REPORTING = 16;
static const int EEPROM_ANALOG_REFERENCE = 17;
static const int EEPROM_INSTANT_DIGITAL_REPORTING = 18;

static const int EEPROM_DIGITAL_INPUTS_TO_REPORT = 50; // size required: TOTAL_PORTS x 1 byte
static const int EEPROM_PORT_CONFIG_INPUTS = 100; // size required: TOTAL_PORTS x 1 byte
static const int EEPROM_PIN_MODES = 150; // TOTAL_PINS x 1 byte

static const int EEPROM_IS_I2C_ENABLED = 200;
static const int EEPROM_I2C_QUERY_INDEX = 201;
static const int EEPROM_I2C_QUERY = 202; // sizeof(i2c_device_info)*queryIndex

static const byte IS_CONFIGURED = 0b10101010;
static const byte CONFIG_VERSION = 7;



#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting


#define ANALOG_PINS 6

static const int ANALOG_SAMPLING = 50;
unsigned long analogPinData[ANALOG_PINS];
unsigned long analogDataCount = 0;

byte analogReferenceVar = DEFAULT; 

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence. 1 port == 8 pins.
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS];  // each bit: 1 = pin in (any) INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis = 0;        // store the current value from millis()
unsigned long previousMillis = 0;       // for comparison with currentMillis
unsigned long previousAnalogMillis = 0;
unsigned long lastSerialMillis = 0;       // for comparison with currentMillis
int lcdInterval = 4000; // TODO: make configurable


unsigned int samplingInterval = DEFAULT_SAMPLING_INTERVAL; // how often to run the main loop (in ms)

unsigned long blinkMillis=0;       // for comparison with currentMillis


/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[64];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

boolean isResetting = false;

// Forward declare a few functions to avoid compiler errors with older versions
// of the Arduino IDE.
void setPinModeCallback(byte, int);
void reportAnalogCallback(byte analogPin, int value);
void sysexCallback(byte, byte, byte*);

/* utility functions */
void wireWrite(byte data)
{
  Wire.write((byte)data);
}

byte wireRead(void)
{
  return Wire.read();
}

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024. [default:64]
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024. [default:64]
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 5, 6 cause the delay() and millis() functions to stop working. 
 *     Other timing-related functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  isI2CEnabled = true;
  EEPROM.update(EEPROM_IS_I2C_ENABLED, isI2CEnabled);

  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
  EEPROM.update(EEPROM_IS_I2C_ENABLED, isI2CEnabled);
  EEPROM.update(EEPROM_I2C_QUERY_INDEX, queryIndex);
}

void readAndReportData(byte address, int theRegister, byte numBytes, byte stopTX) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    wireWrite((byte)theRegister);
    Wire.endTransmission(stopTX); // default = true
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available()) {
    dbg("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    dbg("I2C: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    i2cRxData[2 + i] = wireRead();
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void sendVirtualWireDigitalOutput(byte portNumber, byte portValue)
{
  #ifdef VIRTUALWIRE
  if(!vwTxPin)
    return;

  byte data[6];
  data[0] = homeId; 
  data[1] = deviceId; // sender
  data[2] = BROADCAST_RECIPIENT;
  data[3] = VIRTUALWIRE_DIGITAL_BROADCAST; 
  data[4] = portNumber;
  data[5] = portValue;
  blink();
  vw_send(data, sizeof(data));
  #endif
}

void sendVirtualWireAnalogOutput(byte pinNumber, int analogData)
{
  #ifdef VIRTUALWIRE
  if(!vwTxPin)
    return;

  byte data[7];
  data[0] = homeId; 
  data[1] = deviceId; // sender
  data[2] = BROADCAST_RECIPIENT;
  data[3] = VIRTUALWIRE_ANALOG_BROADCAST; 
  data[4] = pinNumber;
  data[5] = analogData >> 8; // msb
  data[6] = analogData;      // lsb
  blink();
  vw_send(data, sizeof(data));
  #endif  
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    if(serialEnabled)
      Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
    #ifdef VIRTUALWIRE
    sendVirtualWireDigitalOutput(portNumber, portValue);
    #endif
    digitalOutputMillis = currentMillis;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(bool force)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), force);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), force);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), force);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), force);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), force);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), force);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), force);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), force);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), force);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), force);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), force);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), force);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), force);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), force);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), force);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), force);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
    if(!isResetting)
      EEPROM.update(EEPROM_PORT_CONFIG_INPUTS + pin/8, portConfigInputs[pin/8]);
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        if(PIN_TO_DIGITAL(pin)==3 || PIN_TO_DIGITAL(pin)==11)
          setPwmFrequency(pin, 1);
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      dbg("Unknown pin mode");
      return;
  }
  dbgf("W: Pin %d mode changed to %d", pin, mode);
  EEPROM.update(EEPROM_PIN_MODES + pin, mode);
}

void configureLcd()
{
  #ifdef LIQUIDCRYSTAL
  if(lcd)
  {
    lcd = NULL;
  } 
  if(lcdPort)
  {
    dbgf("LCD %d %d %d", lcdPort, lcdColumns, lcdRows);
    lcd0 = LiquidCrystal_I2C(lcdPort, lcdColumns, lcdRows);
    lcd = &lcd0;
    lcd->begin();
    lcd->print("AutomateFirmata");
    lcd->setCursor(0,1);
    lcd->print("     ready!");
    lcd->setCursor(0,0);
    lcd->setBacklight(false);
  }
  #endif
}

void configureVirtualWire()
{
  #ifdef VIRTUALWIRE
  dbgf("VW: %d %d %d %d", vwRxPin, vwTxPin, vwPttPin, virtualWireSpeed);
  if(virtualWireSpeed < 1 || virtualWireSpeed > 9)
    virtualWireSpeed = DEFAULT_VIRTUALWIRE_SPEED;
  vw_rx_stop();
  vw_tx_stop();
  if(vwRxPin)
    vw_set_rx_pin(vwRxPin);
  if(vwTxPin)
    vw_set_tx_pin(vwTxPin);
  if(vwPttPin)
    vw_set_ptt_pin(vwPttPin);
  if(wakeUpPin && !(portConfigInputs[wakeUpPin/8] && (1<<wakeUpPin))) // if not already configured as input 
    pinMode(wakeUpPin, INPUT);
  if(virtualWireSpeed && (vwTxPin || vwRxPin))
  {
    vw_setup(1000*virtualWireSpeed);
    if(vwRxPin)
      vw_rx_start();
  }
  #endif
}

/*
 * Sets the value of an individual pin. Useful if you want to set a pin value but
 * are not tracking the digital port state.
 * Can only be used on pins configured as OUTPUT.
 * Cannot be used to enable pull-ups on Digital INPUT pins.
 */
void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (Firmata.getPinMode(pin)) {
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        Firmata.setPinState(pin, value);
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          if (Firmata.getPinMode(pin) == OUTPUT) {
            pinWriteMask |= mask;
          } else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
            // only handle INPUT here for backwards compatibility
            pinMode(pin, INPUT_PULLUP);
          }
          Firmata.setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  int analogData;
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        analogData = analogRead(analogPin);
        if(serialEnabled)
          Firmata.sendAnalog(analogPin, analogData);
        sendVirtualWireAnalogOutput(analogPin, analogData);
      }
    }
    if(!isResetting)
      EEPROM.put(EEPROM_ANALOG_INPUTS_TO_REPORT, analogInputsToReport);
  }
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = value;
    if(!isResetting)
      EEPROM.update(EEPROM_DIGITAL_INPUTS_TO_REPORT + port, value);
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;
  byte lcdCommand;

  switch (command) {
    case SYSEX_SETUP_LCD:
      lcdPort = argv[0];
      lcdColumns = argv[1];
      lcdRows = argv[2];
      lcdReporting = argv[3];
      EEPROM.update(EEPROM_LCD_PORT, lcdPort);
      EEPROM.update(EEPROM_LCD_COLUMNS, lcdColumns);
      EEPROM.update(EEPROM_LCD_ROWS, lcdRows);
      EEPROM.update(EEPROM_LCD_REPORTING, lcdReporting);
      configureLcd();
      break;
    case SYSEX_LCD_COMMAND:
      #ifdef LIQUIDCRYSTAL
      if(!lcd)
        return;
      
      lcdCommand = argv[0];
      switch (lcdCommand) {
        case LCD_SET_BACKLIGHT:
          if(argv[1])
            lcd->backlight();
          else
            lcd->noBacklight();
          break;
        case LCD_SET_CURSOR:
          lcd->setCursor(argv[1], argv[2]);
          break;
        case LCD_CLEAR:
          lcd->clear();
          break;
        case LCD_PRINT:
          lcd->print((char*)&argv[1]);
          break;
        case LCD_SET_REPORTING:
          lcdReporting = argv[1];
          EEPROM.update(EEPROM_LCD_REPORTING, lcdReporting);
          break;
      }
      #endif
      break;
    case SYSEX_KEEP_ALIVE:
      dbg("I'm alive!");
      break;
    case SYSEX_SETUP_VIRTUALWIRE:
      vwRxPin = argv[0];
      vwTxPin = argv[1];
      vwPttPin = argv[2];
      wakeUpPin = argv[3],
      virtualWireSpeed = argv[4];
      homeId = argv[5];
      deviceId = argv[6];

      EEPROM.update(EEPROM_VIRTUALWIRE_RX_PIN, vwRxPin);
      EEPROM.update(EEPROM_VIRTUALWIRE_TX_PIN, vwTxPin);
      EEPROM.update(EEPROM_VIRTUALWIRE_PTT_PIN, vwPttPin);
      EEPROM.update(EEPROM_WAKEUP_PIN, wakeUpPin);
      EEPROM.update(EEPROM_VIRTUALWIRE_SPEED, virtualWireSpeed);
      EEPROM.update(EEPROM_HOME_ID, homeId);
      EEPROM.update(EEPROM_DEVICE_ID, deviceId);
      configureVirtualWire();
      break;
    case SYSEX_SET_ANALOG_REFERENCE:
        analogReferenceVar = argv[0];
        EEPROM.update(EEPROM_ANALOG_REFERENCE, analogReferenceVar);
        analogReference(analogReferenceVar);
        break;
    case SYSEX_SET_INSTANT_DIGITAL_REPORTING:
        instantDigitalReporting = argv[0];
        EEPROM.update(EEPROM_INSTANT_DIGITAL_REPORTING, instantDigitalReporting);
        break;
    case SYSEX_VIRTUALWIRE_MESSAGE:
      #ifdef VIRTUALWIRE
      blink();
      if(vwTxPin)
        vw_send(argv, argc);
      #endif
      break;
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        dbg("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      // need to invert the logic here since 0 will be default for client
      // libraries that have not updated to add support for restart tx
      if (argv[1] & I2C_END_TX_MASK) {
        stopTX = I2C_RESTART_TX;
      }
      else {
        stopTX = I2C_STOP_TX; // default
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
            wireWrite(data);
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data, stopTX);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            // too many queries, just ignore
            dbg("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          query[queryIndex].stopTX = stopTX;
          EEPROM.update(EEPROM_I2C_QUERY_INDEX, queryIndex);
          EEPROM.put(EEPROM_I2C_QUERY + queryIndex*sizeof(i2c_device_info), query[queryIndex]);
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            queryIndexToSkip = 0;
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
                query[i].stopTX = query[i + 1].stopTX;
                EEPROM.put(EEPROM_I2C_QUERY + i*sizeof(i2c_device_info), query[i]);
              }
            }
            queryIndex--;
          }
          EEPROM.update(EEPROM_I2C_QUERY_INDEX, queryIndex);

          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      setSleepMode();
      EEPROM.put(EEPROM_SAMPLING_INTERVAL, samplingInterval);
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;

    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
  }
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void setSleepMode()
{
  #ifdef LOWPOWER
  if(samplingInterval >= 8000) 
  {
    sleepMode = SLEEP_8S;
    sleepTime = 8000;
  }
  else if(samplingInterval >= 4000)
  {
    sleepMode = SLEEP_4S;
    sleepTime = 4000;
  }
  else if(samplingInterval >= 2000)
  {
    sleepMode = SLEEP_2S;
    sleepTime = 2000;
  }
  else if(samplingInterval >= 1000)
  {
    sleepMode = SLEEP_1S;
    sleepTime = 1000;
  }
  else if(samplingInterval >= 500)
  {
    sleepMode = SLEEP_500MS;
    sleepTime = 500;
  }   
  else if(samplingInterval >= 250)
  {
    sleepMode = SLEEP_250MS;
    sleepTime = 250;
  }
  else if(samplingInterval >= 120)
  {
    sleepMode = SLEEP_120MS;
    sleepTime = 120;
  }
  else if(samplingInterval >= 60)
  {
    sleepMode = SLEEP_60MS;
    sleepTime = 60;
  }
  else if(samplingInterval >= 30)
  {
    sleepMode = SLEEP_30MS;
    sleepTime = 30;
  }
  else
  {
    sleepMode = SLEEP_15MS;
    sleepTime = 15;
  }
  #endif
}

void hardReset()
{
  if (isI2CEnabled) {
      disableI2CPins();
  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    dbgf("reset reportPINS[%d]: %d", i, false);
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;

    EEPROM.update(EEPROM_DIGITAL_INPUTS_TO_REPORT + i, (byte)0);
    EEPROM.update(EEPROM_PORT_CONFIG_INPUTS + i, (byte)0);
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output

    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT); 
    }
  }

  analogInputsToReport = 0;
  
  EEPROM.put(EEPROM_ANALOG_INPUTS_TO_REPORT, (int)0);
  
  analogReferenceVar = DEFAULT; 
  analogReference(DEFAULT);
  EEPROM.update(EEPROM_ANALOG_REFERENCE, analogReferenceVar);
  instantDigitalReporting = true;
  EEPROM.update(EEPROM_INSTANT_DIGITAL_REPORTING, instantDigitalReporting);
  #ifdef VIRTUALWIRE
  vw_rx_stop();
  vw_tx_stop();
  vwRxPin = 0;
  vwTxPin = 0;
  vwPttPin = 0;
  wakeUpPin = 0;
  virtualWireSpeed = DEFAULT_VIRTUALWIRE_SPEED;
  samplingInterval = DEFAULT_SAMPLING_INTERVAL;
  #endif
  #ifdef LIQUIDCRYSTAL
  lcd = NULL;
  lcdPort = 0;
  lcdColumns = 0;
  lcdRows = 0;
  lcdReporting = false;
  
  EEPROM.update(EEPROM_LCD_PORT, lcdPort);
  EEPROM.update(EEPROM_LCD_COLUMNS, lcdColumns);
  EEPROM.update(EEPROM_LCD_ROWS, lcdRows);
  EEPROM.update(EEPROM_LCD_REPORTING, lcdReporting);
  #endif
  #ifdef VIRTUALWIRE
  EEPROM.update(EEPROM_VIRTUALWIRE_TX_PIN, vwTxPin);
  EEPROM.update(EEPROM_VIRTUALWIRE_RX_PIN, vwRxPin);
  EEPROM.update(EEPROM_VIRTUALWIRE_PTT_PIN, vwPttPin);
  EEPROM.update(EEPROM_VIRTUALWIRE_SPEED, virtualWireSpeed);
  EEPROM.update(EEPROM_WAKEUP_PIN, wakeUpPin);
  EEPROM.put(EEPROM_SAMPLING_INTERVAL, samplingInterval);
  #endif
  EEPROM.update(EEPROM_CONFIGURED, IS_CONFIGURED);
  EEPROM.update(EEPROM_CONFIG_VERSION, CONFIG_VERSION);
}

void systemResetCallbackFunc(bool init_phase)
{
  isResetting = true;
  bool configRead = false;

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

  if(init_phase)
  {
    configRead = readEepromConfig();
  }

  if(!configRead)
  {
    hardReset();
  }

  setSleepMode();
  isResetting = false;
  dbg("System reset ready");
}

void systemResetCallback()
{
  systemResetCallbackFunc(false);
}

void blink()
{
  digitalWrite(BLINK_PIN, HIGH);
  blinkMillis = millis();
}

inline void readVirtualWire()
{
  #ifdef VIRTUALWIRE
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen))
  {
    dbgf("Vw message %s", buf)
    if (buflen < HEADER_LENGTH + 1) return; // our headers and at least 1 bytes of data

    uint8_t homeAddress = buf[0];
    uint8_t senderAddress = buf[1];
    uint8_t recipientAddress = buf[2];
    uint8_t command = buf[3];

    uint8_t *arg1 = &buf[4];
    uint8_t *arg2 = &buf[5];
    dbgf("VW msg: home: %d sender %d recipient %d arg1 %d arg2 %d",
         homeAddress,
         senderAddress,
         recipientAddress,
         command,
         *arg1,
         *arg2)
    // check correct address and ignore if not ours
    if (homeAddress == homeId && (recipientAddress == deviceId || recipientAddress == BROADCAST_RECIPIENT))
    {
      switch(command) {
        case VIRTUALWIRE_SET_PIN_MODE:
          setPinModeCallback(*arg1, *arg2);
          break;
        case VIRTUALWIRE_ANALOG_MESSAGE:
          analogWriteCallback(*arg1, *arg2);
          break;
        case VIRTUALWIRE_DIGITAL_MESSAGE:
          digitalWriteCallback(*arg1, *arg2);
          break;
        case VIRTUALWIRE_START_SYSEX:
          sysexCallback(arg1, buflen - HEADER_LENGTH - 1, arg2);
          break;
        case VIRTUALWIRE_SET_DIGITAL_PIN_VALUE:
          setPinValueCallback(*arg1, *arg2);
          break;
        case VIRTUALWIRE_DIGITAL_BROADCAST:
        case VIRTUALWIRE_ANALOG_BROADCAST:
          if(serialEnabled)
          {
            Firmata.write(START_SYSEX);
            Firmata.write(SYSEX_VIRTUALWIRE_MESSAGE);
            Firmata.write(senderAddress);
            Firmata.write(command);
            for(int i=0; i < buflen - HEADER_LENGTH; i++)
              Firmata.write(arg1[i]);
            Firmata.write(END_SYSEX);
          }
          break;
      }
    }
  }
  #endif
}

bool readEepromConfig()
{
  uint8_t configured = EEPROM.read(EEPROM_CONFIGURED);
  uint8_t config_version = EEPROM.read(EEPROM_CONFIG_VERSION);

  if(configured != IS_CONFIGURED || config_version != CONFIG_VERSION)
    return false;

  #ifdef VIRTUALWIRE
  homeId = EEPROM.read(EEPROM_HOME_ID);
  deviceId = EEPROM.read(EEPROM_DEVICE_ID);
  EEPROM.get(EEPROM_SAMPLING_INTERVAL, samplingInterval);
  vwRxPin = EEPROM.read(EEPROM_VIRTUALWIRE_RX_PIN);
  vwTxPin = EEPROM.read(EEPROM_VIRTUALWIRE_TX_PIN);
  vwPttPin = EEPROM.read(EEPROM_VIRTUALWIRE_PTT_PIN);
  wakeUpPin = EEPROM.read(EEPROM_WAKEUP_PIN);
  virtualWireSpeed = EEPROM.read(EEPROM_VIRTUALWIRE_SPEED);
  #endif
  isI2CEnabled = EEPROM.read(EEPROM_IS_I2C_ENABLED);  
  queryIndex = (signed char)EEPROM.read(EEPROM_I2C_QUERY_INDEX);  
  
  for(int i=0; i<TOTAL_PINS; i++)
    if(IS_PIN_DIGITAL(i) || IS_PIN_ANALOG(i))
    { 
      setPinModeCallback(i, EEPROM.read(EEPROM_PIN_MODES + i));
    }
  dbg("Reading port config...");
  for(int port=0; port<TOTAL_PORTS; port++)
  {
    reportPINs[port] = EEPROM.read(EEPROM_DIGITAL_INPUTS_TO_REPORT + port);
    portConfigInputs[port] = EEPROM.read(EEPROM_PORT_CONFIG_INPUTS + port); 
  }

  dbg("Reading I2C config...");
  for(int q=0; q<queryIndex; q++)
    EEPROM.get(EEPROM_I2C_QUERY + q*sizeof(i2c_device_info), query[q]);
  
  if(isI2CEnabled)
    enableI2CPins();

  EEPROM.get(EEPROM_ANALOG_INPUTS_TO_REPORT, analogInputsToReport);
  analogReferenceVar = EEPROM.read(EEPROM_ANALOG_REFERENCE);
  analogReference(analogReferenceVar);
  instantDigitalReporting = EEPROM.read(EEPROM_INSTANT_DIGITAL_REPORTING);

  dbg("VW config");
  configureVirtualWire();
  
  dbg("LCD config");
  #ifdef LIQUIDCRYSTAL
  lcdPort = EEPROM.read(EEPROM_LCD_PORT);
  lcdColumns = EEPROM.read(EEPROM_LCD_COLUMNS);
  lcdRows = EEPROM.read(EEPROM_LCD_ROWS);
  lcdReporting = EEPROM.read(EEPROM_LCD_REPORTING);
  configureLcd();
  #endif
  checkDigitalInputs(true);
  return true;
}

void setup()
{ 
  // Disable vw pins initially by setting them to invalid value
  #ifdef VIRTUALWIRE
  vw_set_ptt_pin(255); 
  vw_set_tx_pin(255); 
  vw_set_rx_pin(255);
  #endif

  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);
  setPinModeCallback(BLINK_PIN, PIN_MODE_OUTPUT);

  // to use a port other than Serial, such as Serial1 on an Arduino Leonardo or Mega,
  // Call begin(baud) on the alternate serial port and pass it to Firmata to begin like this:
  // Serial1.begin(57600);
  // Firmata.begin(Serial1);
  // However do not do this if you are using SERIAL_MESSAGE
  Firmata.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }
  systemResetCallbackFunc(true);
  for(int i=0; i<ANALOG_PINS; i++)
    analogPinData[i] = 0;

  dbg("Setup ready.");
}

void wakeUp(){}

void loop()
{

  byte pin, analogPin;
  int analogData;
  unsigned short int pinIdx = 0;
  unsigned short int lastPin = 0;

  if(vwTxPin && !vwRxPin && !serialEnabled)
  {
    #ifdef VIRTUALWIRE
    vw_wait_tx();
    #endif
    #ifdef LOWPOWER
    if(wakeUpPin)
      attachInterrupt(0, wakeUp, CHANGE);
    LowPower.powerDown(sleepMode, ADC_OFF, BOD_OFF);
    if(wakeUpPin)
      detachInterrupt(0);
    #endif  
    currentMillis += sleepTime;
  }
  else
    currentMillis = millis();

  bool checkImmediately = instantDigitalReporting || (currentMillis >= digitalOutputMillis + samplingInterval);
  if((!vwTxPin || wakeUpPin) && checkImmediately)
  { 
    checkDigitalInputs(false);
  }
  if(blinkMillis && (currentMillis >= blinkMillis + BLINK_INTERVAL))
  {
    digitalWrite(BLINK_PIN, LOW);
    blinkMillis = 0;
  }

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
  {
    Firmata.processInput();
    lastSerialMillis = currentMillis;
    serialEnabled = true;
  }
  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  if(vwTxPin && serialEnabled && currentMillis >= lastSerialMillis + SERIAL_SHUTDOWN_TIME)
    serialEnabled = false;
  
  if(currentMillis >= previousAnalogMillis + ANALOG_SAMPLING)
  {
    previousAnalogMillis = currentMillis;
    for(analogPin = 0; analogPin < 6; analogPin++)
        if (analogInputsToReport & (1 << analogPin))
          analogPinData[analogPin] += analogRead(analogPin);
    analogDataCount ++; 
  }

  pinIdx = 0;
  if (currentMillis >= previousMillis + samplingInterval) {
    previousMillis = currentMillis;
    if(vwTxPin || !instantDigitalReporting)
      checkDigitalInputs((bool)vwTxPin);
    if(lcd && lcdReporting && (reportPINs[0] || reportPINs[1]))
    {
      for(unsigned short int port=0; port<2; port++)
      {
        byte v = previousPINs[port];
        //dbg("Writing digital to lcd");
        snprintf_P(lcdBuf, sizeof(lcdBuf), PSTR("D%2d-%2d: %d%d%d%d%d%d%d%d"),  
                   port*8, 
                   (port+1)*8-1,
                   bool(v & (1<<0)),
                   bool(v & (1<<1)),
                   bool(v & (1<<2)),
                   bool(v & (1<<3)),
                   bool(v & (1<<4)),
                   bool(v & (1<<5)),
                   bool(v & (1<<6)),
                   bool(v & (1<<7))
                );
        #ifdef LIQUIDCRYSTAL        
        if(reportPINs[0] && pinIdx == reportPin)
        {
          lcd -> setCursor(0,0);
          lcd -> print(lcdBuf);
        }
        else if(reportPINs[1] && pinIdx == reportPin + 1)
        {
          lcd -> setCursor(0,1);
          lcd -> print(lcdBuf);
        }
        #endif
        pinIdx ++;
      }
      pinIdx += 2;
    }
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          if(analogPin < ANALOG_PINS && analogDataCount)
            analogData = analogPinData[analogPin] / analogDataCount;
          else
            analogData = analogRead(analogPin);
            
          if(serialEnabled)
            Firmata.sendAnalog(analogPin, analogData);
          sendVirtualWireAnalogOutput(analogPin, analogData);
          if(lcd && lcdReporting)
          {
            float f = 0.999999*float(analogData)/1023.;
            snprintf_P(lcdBuf, sizeof(lcdBuf), PSTR("A%d:%d.%02d "), analogPin, (int)f, (int)(f*100)%100);
            #ifdef LIQUIDCRYSTAL
            if(pinIdx == reportPin)
            {
              lcd -> setCursor(0,0);
              lcd -> print(lcdBuf);
            }
            else if(pinIdx == reportPin + 1)
            {
              lcd -> setCursor(8,0);
              lcd -> print(lcdBuf);
            }
            else if(pinIdx == reportPin + 2)
            {
              lcd -> setCursor(0,1);
              lcd -> print(lcdBuf);
            }
            
            else if(pinIdx == reportPin + 3)
            {
              lcd -> setCursor(8,1);
              lcd -> print(lcdBuf);
            }
            #endif
          }
          pinIdx++;
        }
      }
    }
    for(int i=0; i<ANALOG_PINS; i++)
      analogPinData[i] = 0;
    dbgf("Analog data count: %d", analogDataCount);
    analogDataCount = 0;
    
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
      }
    }
    lastPin = pinIdx - 1;
    if(currentMillis > previousLCDMillis + lcdInterval)
    {
      previousLCDMillis = currentMillis;
      reportPin += 4;
      if(reportPin > lastPin)
        reportPin = 0;
    }
  }
    
  if(vwRxPin)
    readVirtualWire(); 
  

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.update();
#endif
}
