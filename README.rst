Firmata sketch for Arduino with additional features to improve co-operation with Automate
=========================================================================================

See Automate at <https://github.com/tuomas2/automate> and 
`documentation <http://python-automate.readthedocs.io/en/latest/official_extensions/arduino.html>`_.

Interesting features on top of StandardFirmata:
 - Pin modes and other configuration is saved to and read from EEPROM
 - VirtualWire support added which enables wireless reading&writing with cheap RF modules. 
   (VirtualWire uses Timer1, so don't use PWM on ports 9,11) 
 - Power saving feature when using only VirtualWire transmitter. 
   Configurable wakeup pin.
 - Simple configuration via SysEx message (which `Automate <https://github.com/tuomas2/automate>`_ implements).

Please also note:
 - Servo support removed (conflicts with VirtualWire due to use of Timer1).
 - If PWM is used on Pins 3 or 11, timer2 divider is set to 1, resulting in 31.25 KHz PWM frequency. 

Requires the following libraries:
 - VirtualWire: http://www.airspayce.com/mikem/arduino/VirtualWire/VirtualWire-1.27.zip 
   (see http://www.airspayce.com/mikem/arduino/VirtualWire/)
 - Low-Power: https://github.com/rocketscream/Low-Power (tested at commit 530e8e7)

Compilation tested to be working fine Arduino IDE version 1.8.3.

Work is based on StandardFirmata 
https://github.com/firmata/arduino/blob/master/examples/StandardFirmata/StandardFirmata.ino @ 281b99f  
