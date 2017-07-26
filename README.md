Firmata with additional features and changes related to improve co-operation with Automate
==========================================================================================

See automate at https://github.com/tuomas2/automate

 - VirtualWire support added. (Uses Timer1, so don't use PWM on ports 9,11) 
 - Servo support removed (conflicts with VirtualWire due to use of Timer1).
 - If PWM is used on Pins 3/11, timer2 divider is set to 1, resulting in 31.25 KHz PWM frequency. 

Requires:
 - VirtualWire: https://github.com/danielesteban/ArduinoLib/tree/master/VirtualWire @ eec925d  
 - Low-Power: https://github.com/rocketscream/Low-Power @ 530e8e7

Work is based on StandardFirmata 
https://github.com/firmata/arduino/blob/master/examples/StandardFirmata/StandardFirmata.ino @ 281b99f  
