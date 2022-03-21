#!/usr/bin/env python

import RPi.GPIO as GPIO
import serial
import time
import board
import adafruit_dht
import urllib.request
import logging
import board
import neopixel
import re


import paho.mqtt.client as mqtt


from ina219 import INA219
from ina219 import DeviceRangeError

#ser = serial.Serial('/dev/ttyUSB3',115200)
#ser.flushInput()

power_key = 6
rec_buff = ''
rec_buff2 = ''
time_count = 0
last_position = ''
last_lat = 0
last_long = 0

# Set the constants that were calculated
SHUNT_OHMS = 0.0015
MAX_EXPECTED_AMPS = 50

# Initial the dht device, with data pin connected to:
#dhtDeviceInside = adafruit_dht.DHT22(board.D18)

# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D21
# The number of NeoPixels
num_pixels = 30
dimmer = 100
last_color = [255, 255, 255]

# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)


last_temp_inside = 0
last_humid_inside = 0
last_current = 0
last_voltage = 0
last_power = 0

start = time.time()
firstrun = True


# you can pass DHT22 use_pulseio=False if you wouldn't like to use pulseio.
# This may be necessary on a Linux single board computer like the Raspberry Pi,
# but it will not work in CircuitPython.
# dhtDevice = adafruit_dht.DHT22(board.D18, use_pulseio=False)



def read():
	# Instantiate the ina object with the above constants
    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
	# Configure the object with the expected bus voltage
	# (either up to 16V or up to 32V with .RANGE_32V)
	# Also, configure the gain to be GAIN_2_80MW for the above example
    ina.configure(ina.RANGE_16V, ina.GAIN_2_80MV)

    global last_temp_inside
    global last_humid_inside
    global last_current
    global last_voltage
    global last_power
    global last_position
    global last_lat
    global last_long
    global start
    global firstrun


	# Prints the values to the console
    print("Bus Voltage: %.3f V" % ina.voltage())
    last_voltage = ina.voltage()
    try:
        last_current = ina.current()/1000
        last_power = ina.power()/1000
        print("Bus Current: %.3f mA" % ina.current())
        print("Power: %.3f mW" % ina.power())
        print("Shunt voltage: %.3f mV" % ina.shunt_voltage())
        print("")
        print("")
    except DeviceRangeError as e:
        print("Current overflow")

    try:
        # Print the values to the serial port
        temperature_c = dhtDeviceInside.temperature
        humidity = dhtDeviceInside.humidity
        print(
            "Temp: {:.1f} C    Humidity: {}% ".format(
                temperature_c, humidity
            )
        )
        last_temp_inside = temperature_c
        last_humid_inside = humidity
        print("")

    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])
        print("")
    except Exception as error:
        dhtDeviceInside.exit()
        raise error





    # SEND IT - to MQTT
    client.publish("snowball/sensor/temp_inside",last_temp_inside)
    client.publish("snowball/sensor/humidity_inside",last_humid_inside)

    client.publish("snowball/sensor/battery_voltage", last_voltage)
    client.publish("snowball/sensor/battery_current", last_current)
    client.publish("snowball/sensor/battery_power", last_power)

    if firstrun == False:
        client.publish("snowball/position/lat_long", str(last_lat) + "," + str(last_long) + ",0.0000000,0.0")

    done = time.time()
    elapsed = done - start
    #print(elapsed)
    if elapsed >= 3600 or firstrun == True:
        firstrun = False
        start = time.time()

        try:
            power_on(power_key)
            get_gps_position()
            power_down(power_key)
        except:
        	if ser != None:
        		ser.close()
        	power_down(power_key)
        	GPIO.cleanup()
        if ser != None:
        		ser.close()
        		GPIO.cleanup()

        last_lat = float(last_position[25:36])/100
        last_long = float(last_position[39:51])/100

        print("------------- Update thingspeak ------------- ")
        if(connect()):
            print("------------- send via wifi -------------")


            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1=" + str(last_humid_inside))
            time.sleep(5)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field2=" + str(last_temp_inside))
            time.sleep(5)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field3=" + str(last_current))
            time.sleep(5)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field4=" + str(last_voltage))
            time.sleep(5)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field5=" + str(last_power))
            time.sleep(5)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field6=" + str(last_lat))
            time.sleep(5)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field7=" + str(last_long))
        else:
            print("send via SIM")
            #send_at('AT+CSQ','OK',1)
            #send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            #send_at('AT+HTTPINIT','OK',1)
            #send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1="','OK',1)
            #send_at('AT+HTTPACTION=0 ','OK',1)




# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected to Broker with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("snowball/light/set_color")
    client.subscribe("snowball/light/dimmer")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print("Received Message: " + msg.topic+" "+str(msg.payload))
    if msg.topic == 'snowball/light/set_color':
        setLightColor(str(msg.payload)[3:9])
    elif msg.topic == 'snowball/light/dimmer':
        setLightDimmer(str(msg.payload))

def hex_to_rgb(hex):
  rgb = []
  for i in (0, 2, 4):
    decimal = int(hex[i:i+2], 16)
    rgb.append(decimal)

  return rgb[0], rgb[2], rgb[1]


def setLightColor(color_hex):
    global last_color
    last_color = hex_to_rgb(color_hex)
    pixels.fill(last_color)
    # Uncomment this line if you have RGBW/GRBW NeoPixels
    # pixels.fill((255, 0, 0, 0))
    pixels.show()
    time.sleep(1)

def setLightDimmer(dimmer):
    global last_color
    new_value = int(re.findall(r"'(.*?)'", dimmer)[0])/100
    dimmed_color = [int(last_color[0]*new_value), int(last_color[1]*new_value), int(last_color[2]*new_value)]
    pixels.fill(dimmed_color)
    pixels.show()
    #time.sleep(1)

def connect(host='http://google.com'):
    try:
        urllib.request.urlopen(host) #Python 3.x
        return True
    except:
        return False

def power_on(power_key):
	print('SIM7600X is starting:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	ser.flushInput()
	print('SIM7600X is ready')


def send_at(command,back,timeout):

    global last_position
    rec_buff = ''
    ser.write((command+'\r\n').encode())
    time.sleep(timeout)
    if ser.inWaiting():
        time.sleep(0.01 )
        rec_buff = ser.read(ser.inWaiting())
        if rec_buff != '':
            if back not in rec_buff.decode():
                #print(command + ' ERROR')
                print(command + ' back:\t' + rec_buff.decode())
                return 0
            else:
                last_position = rec_buff.decode()
                print(last_position)
                return 1
        else:
            print('Modem is not ready')
            return 0

#+CGPSINFO: 4804.512069,N,01114.890322,E,180322,141108.0,599.7,0.0,

def get_gps_position():
    rec_null = True
    answer = 0
    print('Start GPS session...')
    rec_buff = ''
    send_at('AT+CGPS=1,1','OK',1)
    time.sleep(2)
    while rec_null:

        answer = send_at('AT+CGPSINFO','+CGPSINFO: ',1)
        if 1 == answer:
            answer = 0
            if ',,,,,,' in rec_buff:
                print('GPS is not ready')
                rec_null = True
                time.sleep(1)
            else:
                print("GPS OK")
                rec_null = False
        else:
            print('error %d'%answer)
            rec_buff = ''
            send_at('AT+CGPS=0','OK',1)
            return False
        time.sleep(1.5)



def power_down(power_key):
	print('SIM7600X is loging off:')
	print('Good bye')

if __name__ == "__main__":
    print("Starting up...")
    print("Connecting to MQTT Broker...")
    print("")
    client = mqtt.Client("Pi") #create new instance
    client.connect("127.0.0.1", 1883, 60)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()


    print("Startup finished - going into loop")
    while True:
        #read()
        time.sleep(2)
