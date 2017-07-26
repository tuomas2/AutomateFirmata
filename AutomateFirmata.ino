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

  Last updated October 16th, 2016

TODO

 - Using LowPower.h, we could try to save some power if serial and vw_rx_pin are disabled.
   Also otherwise we could check which features of chip are needed / in use.

*/




// Device specific configuraiton

#include <VirtualWire.h>
#include <Wire.h>
#include <Firmata.h>
#include <EEPROM.h>
#include <LowPower.h>

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
#define BLINK_INTERVAL 0
#define BLINK_PIN 13
#define SERIAL_SHUTDOWN_TIME 120000 // 2 minutes

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

#define VIRTUALWIRE_BAUDRATE 2000 // setting 9600 breaks mysteriously other other input reading

char tmpbuf[128];

uint8_t home_id = 0x01; // Set this different to your neighbors 
uint8_t device_id = 0x01; // Set this individual within your home

uint8_t vw_rx_pin = 0; // 0 disabled, otherwise pin number
uint8_t vw_tx_pin = 0;

boolean serial_enabled = true;

static const int BROADCAST_RECIPIENT = 0xFF;
static const uint8_t HEADER_LENGTH = 4;

static const byte PIN_MODE_VIRTUALWIRE_WRITE = 0x0C;
static const byte PIN_MODE_VIRTUALWIRE_READ = 0x0D;
// Outgoing sysex's
static const int SYSEX_DIGITAL_PULSE = 0x91;

// Incoming sysexs (0x00-0x0F are user defined according to FirmataConstants.h, let's use those)
static const byte SYSEX_VIRTUALWIRE_MESSAGE = 0x01;
static const byte SYSEX_SET_IDENTIFICATION = 0x02;
static const byte SYSEX_KEEP_ALIVE = 0x03;


// Virtualwire command bytes
static const byte VIRTUALWIRE_SET_PIN_MODE = 0x01;
static const byte VIRTUALWIRE_ANALOG_MESSAGE = 0x02;
static const byte VIRTUALWIRE_DIGITAL_MESSAGE = 0x03;
static const byte VIRTUALWIRE_START_SYSEX = 0x04;
static const byte VIRTUALWIRE_SET_DIGITAL_PIN_VALUE = 0x05;
static const byte VIRTUALWIRE_DIGITAL_BROADCAST = 0x06;
static const byte VIRTUALWIRE_ANALOG_BROADCAST = 0x07;

// EEPROM addressses
static const int EEPROM_HOME_ID_ADR = 0;
static const int EEPROM_DEVICE_ID_ADR = 1;
static const int EEPROM_VIRTUALWIRE_RX_PIN_ADR = 2;
static const int EEPROM_VIRTUALWIRE_TX_PIN_ADR = 3;
static const int EEPROM_SAMPLING_INTERVAL = 5; // 2 bytes
static const int EEPROM_ANALOG_INPUTS_TO_REPORT = 7; // 2 byte
static const int EEPROM_DIGITAL_INPUTS_TO_REPORT = 10; // size required: TOTAL_PORTS x 1 byte
static const int EEPROM_PORT_CONFIG_INPUTS = 30; // size required: TOTAL_PORTS x 1 byte
static const int EEPROM_PORT_CONFIG_PULL_UPS = 50; // size required: TOTAL_PORTS x 1 byte


#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
boolean reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence. 1 port == 8 pins.
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS];  // each bit: 1 = pin in (any) INPUT, 0 = anything else
byte portConfigPullUps[TOTAL_PORTS]; // each bit: 1 = pin in PULL_UP, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned long lastSerialMillis = 0;       // for comparison with currentMillis

unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

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
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
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

  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
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
    Firmata.sendString("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C: Too few bytes received");
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
   if(!vw_tx_pin)
      return;

   byte data[6];
   data[0] = home_id; 
   data[1] = device_id; // sender
   data[2] = BROADCAST_RECIPIENT;
   data[3] = VIRTUALWIRE_DIGITAL_BROADCAST; 
   data[4] = portNumber;
   data[5] = portValue;
   blink();
   vw_send(data, sizeof(data));
}

void sendVirtualWireAnalogOutput(byte pinNumber, int analogData)
{
   if(!vw_tx_pin)
      return;

   byte data[7];
   data[0] = home_id; 
   data[1] = device_id; // sender
   data[2] = BROADCAST_RECIPIENT;
   data[3] = VIRTUALWIRE_ANALOG_BROADCAST; 
   data[4] = pinNumber;
   data[5] = analogData >> 8; // msb
   data[6] = analogData;      // lsb
   blink();
   vw_send(data, sizeof(data));  
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    if(serial_enabled)
        Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
    sendVirtualWireDigitalOutput(portNumber, portValue);
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
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
    if (mode == PIN_MODE_PULLUP) {
      portConfigPullUps[pin / 8] |= (1 << (pin & 7));
    }
    else
    {
      portConfigPullUps[pin / 8] &= ~(1 << (pin & 7));
    }

    if(!isResetting)
    {
      EEPROM.update(EEPROM_PORT_CONFIG_INPUTS + pin/8, portConfigInputs[pin/8]);
      EEPROM.update(EEPROM_PORT_CONFIG_PULL_UPS + pin/8, portConfigPullUps[pin/8]);
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
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
        if(pin==3 || pin==11)
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
    case PIN_MODE_VIRTUALWIRE_WRITE:
        if (IS_PIN_DIGITAL(pin)) {
            vw_tx_stop();
            vw_set_tx_pin(pin);
            vw_setup(VIRTUALWIRE_BAUDRATE);
            vw_tx_pin = pin;
            EEPROM.update(EEPROM_VIRTUALWIRE_TX_PIN_ADR, pin);
        }
        break;
    case PIN_MODE_VIRTUALWIRE_READ:
        if (IS_PIN_DIGITAL(pin)) {
            vw_rx_stop();
            vw_set_rx_pin(pin);
            vw_setup(VIRTUALWIRE_BAUDRATE);
            vw_rx_start();
            vw_rx_pin = pin;
            EEPROM.update(EEPROM_VIRTUALWIRE_RX_PIN_ADR, pin);
        }
        break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
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
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
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
        if(serial_enabled)
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
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
    if(!isResetting)
      EEPROM.update(EEPROM_DIGITAL_INPUTS_TO_REPORT + port, reportPINs[port]);
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

  switch (command) {
    case SYSEX_KEEP_ALIVE:
        break;
    case SYSEX_VIRTUALWIRE_MESSAGE:
        blink();
        vw_send(argv, argc);
        break;
    case SYSEX_SET_IDENTIFICATION:
        home_id = argv[0];
        device_id = argv[1];
        EEPROM.update(EEPROM_HOME_ID_ADR, home_id);
        EEPROM.update(EEPROM_DEVICE_ID_ADR, device_id);
        break;
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
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
            Firmata.sendString("too many queries");
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
              }
            }
            queryIndex--;
          }
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

void systemResetCallbackFunc(bool init_phase)
{
  isResetting = true;
  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

  if (isI2CEnabled) {
    disableI2CPins();
  }
  
  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    portConfigPullUps[i] = 0;  // until activated
    previousPINs[i] = 0;
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

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  if(init_phase)
    readEepromConfig();
  else
  { 
    // by default, do not report any analog inputs
    analogInputsToReport = 0;
    EEPROM.put(EEPROM_ANALOG_INPUTS_TO_REPORT, (int)0);
    for(int i=0; i<TOTAL_PORTS; i++)
    {
        EEPROM.update(EEPROM_DIGITAL_INPUTS_TO_REPORT + i, (byte)0);
        EEPROM.update(EEPROM_PORT_CONFIG_INPUTS + i, (byte)0);
        EEPROM.update(EEPROM_PORT_CONFIG_PULL_UPS + i, (byte)0);
    }
  } 
  isResetting = false;
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
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen))
  {
    if (buflen < HEADER_LENGTH + 1) return; // our headers and at least 1 bytes of data
    
    uint8_t home_address = buf[0];
    uint8_t sender_address = buf[1];
    uint8_t recipient_address = buf[2];
    uint8_t command = buf[3];

    uint8_t *arg1 = &buf[4];
    uint8_t *arg2 = &buf[5];
    // check correct address and ignore if not ours
    if (home_address == home_id && (recipient_address == device_id || recipient_address == BROADCAST_RECIPIENT))
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
          if(serial_enabled)
          {
            Firmata.write(START_SYSEX);
            Firmata.write(SYSEX_DIGITAL_PULSE);
            Firmata.write(sender_address);
            Firmata.write(command);
            for(int i=0; i < buflen - HEADER_LENGTH; i++)
                Firmata.write(arg1[i]);
            Firmata.write(END_SYSEX);
          }
          break;
        }
      }
    }
}
    
void readEepromConfig()
{
  home_id = EEPROM.read(EEPROM_HOME_ID_ADR);
  device_id = EEPROM.read(EEPROM_DEVICE_ID_ADR);
  EEPROM.get(EEPROM_SAMPLING_INTERVAL, samplingInterval);
  vw_rx_pin = EEPROM.read(EEPROM_VIRTUALWIRE_RX_PIN_ADR);
  vw_tx_pin = EEPROM.read(EEPROM_VIRTUALWIRE_TX_PIN_ADR);

  if(vw_rx_pin)
    setPinModeCallback(vw_rx_pin, PIN_MODE_VIRTUALWIRE_READ);
  
  if(vw_tx_pin)
    setPinModeCallback(vw_tx_pin, PIN_MODE_VIRTUALWIRE_WRITE);

  EEPROM.get(EEPROM_ANALOG_INPUTS_TO_REPORT, analogInputsToReport);
  for(int analog_pin = 0; analog_pin < TOTAL_ANALOG_PINS; analog_pin ++)
      if((1 << analog_pin) & analogInputsToReport)
         setPinModeCallback(analog_pin, PIN_MODE_ANALOG);

  for(int port=0; port<TOTAL_PORTS; port++)
  {
    reportPINs[port] = EEPROM.read(EEPROM_DIGITAL_INPUTS_TO_REPORT + port);
    portConfigInputs[port] = EEPROM.read(EEPROM_PORT_CONFIG_INPUTS + port);
    portConfigPullUps[port] = EEPROM.read(EEPROM_PORT_CONFIG_PULL_UPS + port);

    for(uint8_t bit=0; bit<8; bit++)
        if(portConfigInputs[port] & (1 << bit))
        {
            uint8_t pin_nr = 8*port + bit;
            if(portConfigPullUps[port] & (1 << bit))
                setPinModeCallback(pin_nr, PIN_MODE_PULLUP);
            else
                setPinModeCallback(pin_nr, PIN_MODE_INPUT);
        }
  }
}


void setup()
{
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
  Firmata.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }
  systemResetCallbackFunc(true);  // reset to default config
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
   
  byte pin, analogPin;
  int analogData;
  

  if(!vw_rx_pin && !serial_enabled)
  {
    vw_wait_tx();
    blink();
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    currentMillis += 250;
  }
  else
    currentMillis = millis();
 

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
  {
    Firmata.processInput();
    lastSerialMillis = currentMillis;
    serial_enabled = true;
  }

  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  if(serial_enabled && currentMillis - lastSerialMillis > SERIAL_SHUTDOWN_TIME)
  {
    Firmata.sendString("Disabling serial output");
    serial_enabled = false;
  }
  if(blinkMillis && (currentMillis > blinkMillis + BLINK_INTERVAL))
  {
    digitalWrite(BLINK_PIN, LOW);
    blinkMillis = 0;
  }
  
 
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis = currentMillis;
    /* DIGITALREAD - as fast as possible, check for changes and output them to the
     * FTDI buffer using Serial.print()  */
    checkDigitalInputs();
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          analogData = analogRead(analogPin);
          if(serial_enabled)
             Firmata.sendAnalog(analogPin, analogData);
          sendVirtualWireAnalogOutput(analogPin, analogData);
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
      }
    }
  }
  readVirtualWire(); 
#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.update();
#endif
}
