# Kjell H Andersen
# 2014

# Python script for controlling the BLDC Strip v2 using Chibios firmware through the serial port
# It will only control the LEDs, but may be used as a template for other more useful scripts.
# This script is tested on a RaspberryPi, but it should also work on any computer where pySerial 
# can be installed. You need to change the line that initializes the serial port.


import serial
import sys


def	crc8(data_in):
	CRC8INIT = 0x00
	CRC8POLY = 0x07
	
	crc	=	CRC8INIT
	
	for	data in	data_in:
		bit_counter	=	8
		
		while	bit_counter>0:
			feedback_bit = (crc	^	data)	&	0x01
			
			if (feedback_bit ==	1):
				crc	=	0xFF & (crc	^	CRC8POLY)
			
			crc	=	(crc >>	1) & 0x7F
			
			if (feedback_bit ==	1):
				crc	=	crc	|	0x80
			
			data = data	>> 1
			bit_counter -= 1

	return crc


str	=  bytearray([0x00,0x00,0x00,0x00,0x00,0x00,0x00])
r	=  bytearray([0x00,0x00,0x00,0x00,0x00,0x00,0x00])

if (len(sys.argv) >= 3):
	if (sys.argv[1] == "led1"):
		str[0] = 0xF0
	elif (sys.argv[1] == "led2"):
		str[0] = 0xF1
	elif (sys.argv[1] == "motorstate"):
		str[0] = 0xF2
	elif (sys.argv[1] == "dutycycle"):
		str[0] = 0xF3
	elif (sys.argv[1] == "direction"):
		str[0] = 0xF4
	elif (sys.argv[1] == "positioncontrol"):
		str[0] = 0xF5
	elif (sys.argv[1] == "angle"):
		str[0] = 0xF6
		

	else:
		print "Argument 1 is not valid, use led1 or led2 or motorstate or dutycycle or direction or positioncontrol";
		sys.exit()
		
	if (sys.argv[2] == "on"):
		str[1] = 0xFF
	elif (sys.argv[2] == "off"):
		str[1] = 0x00
	elif (sys.argv[2] == "val"):
		if(len(sys.argv) == 4):
			str[1] = int( (int(sys.argv[3])/256) ) # Signed value
			str[2] = int(int(sys.argv[3])%256)
		else:
			print "Not correct number of arguments. Provide 3 arguments"
			sys.exit()				
	else:
		print "Argument 2 is not valid, use on or off";
		sys.exit()
else:
	print "Not correct number of arguments. Provide 2 arguments"

str.append(crc8 (str))

# Change the device when using this in other platforms than RaspberryPi
# ser = serial.Serial('/dev/ttyAMA0', 9600)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.write (str)
print "Read answer";
r = ser.read(8)
result = [ord(b) for b in r]
motorstate = result[1]
delta_t = (result[2]<<8) + (result[3])
u_dc = result[4]
i_dc = result[5] # *21/13
i_dc_ref = result[6]
print "state=%d; delta_t=%d; u_dc=%d; i_dc=%d; i_dc_ref=%d" % (motorstate, delta_t, u_dc, i_dc, i_dc_ref)