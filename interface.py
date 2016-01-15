#!/usr/bin/python

import serial
import sys
import thread, time
import struct

def printMenu():
    print "1.- Version"
    print "2.- Get ADC"
    print "3.- Direction Left"
    print "4.- Direction Right"
    print "5.- Increase DC"
    print "6.- Decrease DC"
    print "7.- Change P"
    print "8.- Chande I"
    print "9.- Change D"
    print "10.- Get P,I,D"
    return

def input_thread(L):
    raw_input()
    L.append(None)
   
def do_print():# print adc readings until enter is pressed
    L = []
    thread.start_new_thread(input_thread, (L,))
    while 1:
        time.sleep(.1)
        if L: break
        print ser.readline()

def packFloat(value):
    return struct.pack('f', value)
       


p=0.0
i=0.0
d=0.0

#ser = serial.Serial('/dev/ttyACM0', 9600)
#ser.write('0')
#print ser.readline()



while (True):
       printMenu()
       command = input("Select Command ")
       #ser.flushInput()
       if command == 1:
          print "the version is"
          ser.write('0')
          print ser.readline()
       elif command == 2:
	  ser.write('1')
          do_print()
          ser.write('1')
       elif command == 3:
          ser.write('2')
       elif command == 4:
          ser.write('3')
       elif command == 5:
          ser.write('4')
       elif command == 6:
          ser.write('5')
       elif command == 7:
          p = input("Gimme new P ")
          ser.write('6')
          #ser.write(packFloat(p))
          #print struct.unpack('f', ser.readline()[0:4])
          ser.write(struct.pack('<f', p))
          #p3 = "\x66\x66\xf6\x40"
          #ser.write(p3)
          print ser.readline()
       elif command == 8:
          i = input("Gimme new I ")
          ser.write('7')
          #ser.write(packFloat(p))
          #print struct.unpack('f', ser.readline()[0:4])
          ser.write(struct.pack('<f', i))
          #p3 = "\x66\x66\xf6\x40"
          #ser.write(p3)
          print ser.readline()
       elif command == 9:
          d = input("Gimme new D ")
          ser.write('8')
          #ser.write(packFloat(p))
          #print struct.unpack('f', ser.readline()[0:4])
          ser.write(struct.pack('<f', d))
          #p3 = "\x66\x66\xf6\x40"
          #ser.write(p3)
          print ser.readline()
       elif command == 10:
	  ser.write('9')
          print ser.readline()
          print ser.readline()
          print ser.readline()
          print ser.readline()
          print ser.readline()
          print ser.readline()
       elif command == 11:
          ser.write('A')
          do_print()
          #ser.write('A')
      




       
      

	
	
	
