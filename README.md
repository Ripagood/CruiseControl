# CruiseControl
C program to control a DC motor using a Timer and controlling via serial with a Python script

Use the DC motor with an H-bridge, either in IC form or made with MOSFETS.

Just run the Python script in a Linux system while having the board connnected through serial communication.

To know what port you should use open a terminal and run

dmesg | grep tty

If connected through USB you should see something like ttyACM0

This program uses py-serial and an ATmega2560

Version 2 scraps some parts of the program and uses an attiny




