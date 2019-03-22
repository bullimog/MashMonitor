# MashMonitor
Arduino Uno Project for controlling Mash Temperature.
Can control power to two electric boilers. As boilers are usually both > 2KW, this until ensures that only one boiler received power at any one time. This prevents power overload into a single socket, and blowing a 13A 240V fuse.


## Controls/Reads the following devices:
* Liquor tank thermometer (Dallas DS18B20) 
* Mash tun thermometer (Dallas DS18B20) 
* 0-5v Out, to control 240v AC Triac
* 12v out, to control 240v Relay, in order to switch power between two electric boilers
* I2C 2004 LCD Panel
* Mechanical Rotary Encoder
* Push button


## Application has eight modes:
l. Off
l. Manually control power to liquor tank.
l. Manually control power to mash tun.
l. Auto control temperature in liquor tank.
l. Auto control temperature in mash tun.
l. Auto control temperature in both liquor tank and mash tun, with liquor tank as the priority.
l. Auto control temperature in both liquor tank and mash tun, with mash tun as the priority.
l. Set temperature calibration.

