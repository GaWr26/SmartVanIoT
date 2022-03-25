#!/usr/bin/env python

import RPi.GPIO as GPIO
import serial
import time
import board
import urllib.request
import logging
import board
import neopixel
import re
import paho.mqtt.client as mqtt
import adafruit_ahtx0


from ina219 import INA219
from ina219 import DeviceRangeError
from pigpio_dht import DHT11, DHT22
from mpu6050 import mpu6050

ser = serial.Serial('/dev/ttyUSB3',115200)
ser.flushInput()

power_key = 6
rec_buff = ''
rec_buff2 = ''
time_count = 0

last_position = ''
last_lat = 0
last_long = 0
last_acc_x = 0
last_acc_y = 0
last_temp_inside = 0
last_humid_inside = 0
last_draw_current = 0
last_draw_voltage = 0
last_draw_power = 0
last_charge_current = 0
last_charge_voltage = 0
last_charge_power = 0

# Set the constants that were calculated
SHUNT_OHMS = 0.0015
MAX_EXPECTED_AMPS = 50

dht_gpio = 18

mpu = mpu6050(0x68)

pixel_pin = board.D21
num_pixels = 30
dimmer = 100
last_color = [255, 255, 255]
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.RGB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)

indoorTempSensor = adafruit_ahtx0.AHTx0(board.I2C())

GPIO.setmode(GPIO.BCM)

RELAIS_1_GPIO = 5
RELAIS_2_GPIO = 6
RELAIS_4_GPIO = 19
RELAIS_3_GPIO = 13
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_2_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_3_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_4_GPIO, GPIO.OUT)

start = time.time()
firstrun = True


# you can pass DHT22 use_pulseio=False if you wouldn't like to use pulseio.
# This may be necessary on a Linux single board computer like the Raspberry Pi,
# but it will not work in CircuitPython.
# dhtDevice = adafruit_dht.DHT22(board.D18, use_pulseio=False)


skipcounter = 0
def read():
	# Instantiate the ina object with the above constants
    ina_draw = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x44)
    ina_charge = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
	# Configure the object with the expected bus voltage
	# (either up to 16V or up to 32V with .RANGE_32V)
	# Also, configure the gain to be GAIN_2_80MW for the above example
    ina_draw.configure(ina_draw.RANGE_16V, ina_draw.GAIN_2_80MV)
    ina_charge.configure(ina_charge.RANGE_16V, ina_charge.GAIN_2_80MV)

    global last_temp_inside
    global last_humid_inside
    global last_draw_current
    global last_draw_voltage
    global last_draw_power
    global last_charge_current
    global last_charge_voltage
    global last_charge_power
    global last_position
    global last_lat
    global last_long
    global start
    global firstrun
    global skipcounter
    global last_acc_x
    global last_acc_y


    print("Draw Bus Voltage: %.3f V" % ina_draw.voltage())
    last_draw_voltage = ina_draw.voltage()
    try:
        last_draw_current = ina_draw.current()/1000
        last_draw_power = (ina_draw.power()/1000) * -1
        print("Draw Bus Current: %.3f mA" % ina_draw.current())
        print("Draw Power: %.3f mW" % (ina_draw.power()* -1))
        print("Draw Shunt voltage: %.3f mV" % ina_draw.shunt_voltage())
        print("")
        print("")
    except DeviceRangeError as e:
        print("Draw Current overflow")


    print("Charge Bus Voltage: %.3f V" % ina_charge.voltage())
    last_charge_voltage = ina_charge.voltage()
    try:
        last_charge_current = ina_charge.current()/1000
        last_charge_power = ina_charge.power()/1000
        print("Charge Bus Current: %.3f mA" % ina_charge.current())
        print("Charge Power: %.3f mW" % ina_charge.power())
        print("Charge Shunt voltage: %.3f mV" % ina_charge.shunt_voltage())
        print("")
        print("")
    except DeviceRangeError as e:
        print("Charge Current overflow")


    print("\nTemperature: %0.1f C" % indoorTempSensor.temperature)
    print("Humidity: %0.1f %%" % indoorTempSensor.relative_humidity)
    last_temp_inside = indoorTempSensor.temperature
    last_humid_inside = indoorTempSensor.relative_humidity

    skipcounter = skipcounter+1

    accel_data = mpu.get_accel_data()
    #print("Acc X : "+str(accel_data['x']))
    #print("Acc Y : "+str(accel_data['y']))
    #print("Acc Z : "+str(accel_data['z']))
    #print()
    gyro_data = mpu.get_gyro_data()
    #print("Gyro X : "+str(gyro_data['x']))
    #print("Gyro Y : "+str(gyro_data['y']))
    #print("Gyro Z : "+str(gyro_data['z']))
    #print()
    #print("-------------------------------")
    last_acc_x = accel_data['x']*100-30+100
    last_acc_y = accel_data['y']*100+30+100


    # SEND IT - to MQTT
    client.publish("snowball/sensor/temp_inside",last_temp_inside)
    client.publish("snowball/sensor/humidity_inside",last_humid_inside)

    client.publish("snowball/sensor/battery_draw_voltage", last_draw_voltage)
    client.publish("snowball/sensor/battery_draw_current", last_draw_current)
    client.publish("snowball/sensor/battery_draw_power", last_draw_power)

    client.publish("snowball/sensor/battery_charge_voltage", last_charge_voltage)
    client.publish("snowball/sensor/battery_charge_current", last_charge_current)
    client.publish("snowball/sensor/battery_charge_power", last_charge_power)

    client.publish("snowball/sensor/accelerometer_x",last_acc_x)
    client.publish("snowball/sensor/accelerometer_y",last_acc_y)

    if firstrun == False:
        client.publish("snowball/position/lat_long", str(last_lat) + "," + str(last_long) + ",0.0000000,0.0")

    done = time.time()
    elapsed = done - start
    #print(elapsed)
    if elapsed >= 3600 or firstrun == True:
        firstrun = False
        start = time.time()

        try:
            get_gps_position()
        except:
        	if ser != None:
        		ser.close()
        	power_down(power_key)
        if ser != None:
        		ser.close()
        try:
            last_lat = float(last_position[25:36])/100
            last_long = float(last_position[39:51])/100
        except:
            print("Noch kein GPS Signal bekommen")

        print("------------- Update thingspeak ------------- ")
        if(connect()):
            print("------------- send via wifi -------------")


            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1=" + str(last_humid_inside))
            time.sleep(1)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field2=" + str(last_temp_inside))
            time.sleep(1)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field3=" + str(last_draw_current))
            time.sleep(1)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field4=" + str(last_draw_voltage))
            time.sleep(1)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field5=" + str(last_draw_power))
            time.sleep(1)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field6=" + str(last_lat))
            time.sleep(1)
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
    client.subscribe("snowball/switch/1")
    client.subscribe("snowball/switch/2")
    client.subscribe("snowball/switch/3")
    client.subscribe("snowball/switch/4")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print("Received Message: " + msg.topic+" "+str(msg.payload))
    if msg.topic == 'snowball/light/set_color':
        setLightColor(str(msg.payload)[3:9])
    elif msg.topic == 'snowball/light/dimmer':
        setLightDimmer(str(msg.payload))
    elif msg.topic == 'snowball/switch/1':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/2':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/3':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/4':
        toggleSwitch(msg.topic[-1], msg.payload)

def toggleSwitch(switch, value):
    new_value = re.findall(r"'(.*?)'", str(value))[0]
    print(switch, new_value)
    if new_value == "on":
        print("switch on")
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.HIGH) # an
    else:
        print("switch off")
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.LOW) # an


def hex_to_rgb(hex):
  rgb = []
  for i in (0, 2, 4):
    decimal = int(hex[i:i+2], 16)
    rgb.append(decimal)

  return tuple(rgb)



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
        read()
        time.sleep(2)
