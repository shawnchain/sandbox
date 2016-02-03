$Id$
Hello Git !

eHacks
======
ILI932X - ILI9320/ILI9325/SSD1289 TFT/LCD Driver for RaspberryPi via GPIO connectors

iOSBLESerial
============

An iOS class simplify the code work for BLE serial communication, without knowing too much of the CoreBluetooth framework and concepts.

Released under GPLv2 license

iBorer
=======
My early experiments on Springboard dylib injection in 2008 which is early than the mobilesubstrate lib. iBorer framework implements a plugin and messaging mechanism and supports dynamic load/unload plugins into springboard.  

The name iBorer means the worm diging into an apple.

Based on the iBorer framework, we can implemented applications such as SMS filter/firewall and also we could define some rules and change them dynamically.

	ifilter -t sms -from 138xxx -j drop
	ifilter -t sms -contains "xyz" -j archive
	ifilter -t call -from 138xxx -time-between 11:00-9:00 -j reject
	ifilter -t [table] [[match rule] AND|OR [match rule]*] [target]
	ifilter -t [table] -p [policy]
	ifilter -t [table] -l
	ifilter -L

All code tested on Jailbroken iPhone with iOS 1/2/3 and now for reference only. 

Have fun. 


