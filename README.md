Standard Firmata with the following special features
====================================================

 - VirtualWire support added. 
   Configuration with pin modes: 0x0C for writing, 0x0D for reading.
   Messages can be sent with SysEx messages (command 0x80), received messages will be pushed back as SysEx messages (0x91). 
 - Servo support removed (conflicts with VirtualWire due to use of Timer1).
 - If PWM is used on Pins 9/10, timer1 divider is set to 1, resulting in 31.25 KHz PWM frequency. 
   (VirtualWire can't be used together with pins in ports 9/10).


Requires VirtualWire: https://github.com/danielesteban/ArduinoLib/tree/master/VirtualWire
Is based on StandardFirmata https://github.com/firmata/arduino/blob/master/examples/StandardFirmata/StandardFirmata.ino @ 281b99f  
