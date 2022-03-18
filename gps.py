#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO

import serial
import time

ser = serial.Serial('/dev/ttyS0',115200)
ser.flushInput()

power_key = 6
rec_buff = ''
rec_buff2 = ''
time_count = 0
last_msg = ''


def power_on(power_key):
	print('SIM7600X is starting:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	#GPIO.setup(power_key,GPIO.OUT)
	#time.sleep(0.1)
	#GPIO.output(power_key,GPIO.HIGH)
	#time.sleep(2)
	#GPIO.output(power_key,GPIO.LOW)
	#time.sleep(20)
	ser.flushInput()
	print('SIM7600X is ready')


def send_at(command,back,timeout):
    rec_buff = ''
    ser.write((command+'\r\n').encode())
    time.sleep(timeout)
    if ser.inWaiting():
        time.sleep(0.01 )
        rec_buff = ser.read(ser.inWaiting())
        if rec_buff != '':
            if back not in rec_buff.decode():
                print(command + ' ERROR')
                print(command + ' back:\t' + rec_buff.decode())
                return 0
            else:
                last_msg = rec_buff.decode()
                print("Response: " + last_msg)
                return 1
        else:
            print('Modem is not ready')
            return 0

def send_gps_position():
    answer = 0
    print('Start GPS session...')
    send_at('AT+CGPS=1,1','OK',1)
    time.sleep(2)
    answer = send_at('AT+CGPSINFO','+CGPSINFO: ',1)
    #time.sleep(4)
    gpsdata = last_msg
    send_at('AT+CSQ','OK',1)
    send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
    send_at('AT+HTTPINIT','OK',1)
    send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1="','OK',1)
    send_at('AT+HTTPACTION=0 ','OK',1)

    print("GPS Data: " + gpsdata)

def power_down(power_key):
	print('SIM7600X is loging off:')
	#GPIO.output(power_key,GPIO.HIGH)
	#time.sleep(3)
	#GPIO.output(power_key,GPIO.LOW)
	#time.sleep(18)
	print('Good bye')

try:
	power_on(power_key)
	send_gps_position()
	power_down(power_key)
except:
	if ser != None:
		ser.close()
	power_down(power_key)
	GPIO.cleanup()
if ser != None:
		ser.close()
		GPIO.cleanup()
