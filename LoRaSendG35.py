#!/usr/bin/python
# -*- coding: utf-8 -*-
import serial
import os
import time
import re
import json
import packer
import socket
import g3
from g3 import g3sensor
from datetime import datetime
from time import sleep
import Adafruit_DHT

BAUDRATE = 57600 # the baud rate we talk to the microchip RN2483
RETRY = 5
MAX_PAYLOAD_LENGTH = 121

#
# start here
#
try:
        input = raw_input
except NameError:
        pass

serial_port = "/dev/ttyUSB0"

# open up the FTDI serial port to get data transmitted to lora
ser = serial.Serial(serial_port, BAUDRATE)

if ser.isOpen() == False:
        ser.open()

# The default settings for the UART interface are
#57600 bps, 8 bits, no parity, 1 Stop bit, no flow control.
ser.bytesize = 8
ser.parity = "N"
ser.stopbits = 1
ser.timeout = 5

print('----------------------------------')
print('cmd> radio cw off')
ser.write(b'radio cw off\r\n')
print(str(ser.readline()))

# signed decimal number representing the transceiver output power, from -3 to 15.
print('cmd> radio set pwr 14')
ser.write(b'radio set pwr 14\r\n')
print(str(ser.readline()))

# decimal representing the operating radio bandwidth, in kHz.
#Parameter values can be: 125, 250, 500.
print('cmd> radio set bw 250')
ser.write(b'radio set bw 250\r\n')
print(str(ser.readline()))

# decimal representing the frequency,
#from 433000000 to 434800000 or from 863000000 to 870000000, in Hz.
print('cmd> radio set freq 868100000')
ser.write(b'radio set freq 868100000\r\n')
print(str(ser.readline()))

# pauses the LoRaWAN stack functionality to allow transceiver
#(radio) configuration must be called before  any radio transmission or reception
print('cmd> mac pause')
ser.write(b'mac pause\r\n')
print(str(ser.readline()))
print('----------------------------------')

try:
        while True:
                time.sleep(8)
                air = g3sensor()
                pmdata = 0
                pm25 = ""
                pm25b = pm25
                try:
                        for i in range(0, 3):
                                pmdata = air.read("/dev/ttyAMA0")
                                time.sleep(1)
                                if pmdata != None:
                                        pm25 = str(pmdata[5])
                                        break
                except:
                        pm25 = pm25b

                sensor = Adafruit_DHT.DHT22
                pin = 4
                humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
                time.sleep(0.5)
                if humidity is not None and temperature is not None:
                        if type(humidity) != 'str' and type(temperature) != 'str':
                                msg = 'Id = 002, Temp={0:0.1f}'.format(temperature)+'C, Humid$
                rawinput = msg
                print msg
                try:
                        byte_rawinput = bytes(rawinput + "\r\n")
                except:
                        byte_rawinput = bytes(rawinput + "\r\n", encoding="UTF-8")

                cmd = "radio tx "
                _length, _payload = packer.Pack_Str(rawinput)
                if int(_length) < int(MAX_PAYLOAD_LENGTH):
                        print("Time: " + str(time.ctime()))
                        byte_rawinput = bytes(cmd + _payload)
                        ser.write(byte_rawinput)
                        ser.readline()
                        time.sleep(0.5 + 0.01 * int(_length))
                        print("Ready to receive ACK")

                        #for i in range(0, RETRY):
                        for i in range(0, 3):
                                ser.write(b'radio rx 0\r\n')
                                ret = ser.readline()
                                ret = ser.readline()
                                ret = ser.readline()

                                if re.match('^radio_rx', str(ret).strip()):
                                        data = ret.split(" ", 1)[1]
                                        _length, _data = packer.Unpack_Str(data)

                                        if _data == packer.ACK:
                                                print("Receive ACK")
                                                break
                                #time.sleep(i)
                                #print("Retry %d", i)
                                #cmd = "radio tx "
                                #_length, _payload = packer.Pack_Str(rawinput)
                                #byte_rawinput = bytes(cmd + _payload)
                                #ser.write(byte_rawinput)
                                #ser.readline()

                                #if i == RETRY - 1:
                                        #ser.flush()
                                        #ser.write(b'sys reset\r\n')
                                        #print(str(ser.readline()))
                                        #ser.flush()
finally:
        ser.close()
        os.system('sudo reboot')
