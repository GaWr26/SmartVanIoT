#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import Adafruit_ADS1x15
import Adafruit_DHT
import sys
import paho.mqtt.client as paho
import board
import neopixel

import DHT22
import pigpio

import os #imports OS library for Shutdown control

# Temperature & humidity Sensors
pi = pigpio.pi()
sensorIndoor = DHT22.sensor(pi, 23)
sensorOutdoor = DHT22.sensor(pi, 24)

# Relais
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # GPIO Nummern statt Board Nummern
RELAIS_MAIN_GPIO = 12
RELAIS_1_GPIO = 7
GPIO.setup(RELAIS_MAIN_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT)
status_fridge = "off"
status_mains = "off"

# Counters
messageCounter = 0
lowVoltageCounter = 0
loopCount = 0
lowVoltageShutdownValue = 12.00

broker = "localhost"

# LED
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D18
# The number of NeoPixels
num_pixels = 53
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=1, auto_write=False, pixel_order=ORDER
)
adc = Adafruit_ADS1x15.ADS1115() # Create an ADS1115 ADC (16-bit) instance

# init
print('')
print('')
print('********************************')
print('SNOWBALL CONTROL starting up....')
print('********************************')
print('')
print('[press ctrl+c to end the script]')
print('')
print('')

# switch on mains on startup
print("Switch on mains")
print('********************************')
print('')
print('')
GPIO.output(RELAIS_MAIN_GPIO, GPIO.HIGH)

# helper to map analog values
def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min
    # Convert the left range into a 0-1 range (int)
    valueScaled = float(value - left_min) / float(left_span)
    # Convert the 0-1 range into a value in the right range.
    return float(right_min + (valueScaled * right_span))

# MQTT callbacks
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected MQTT disconnection. Will auto-reconnect")

def on_connect(client, userdata, flags, rc):
    if rc==0:
        client.connected_flag=True #set flag
        print("MQTT connected OK")
    else:
        print("Bad connection Returned code=",rc)

def on_message(client, userdata, message):
    global status_mains
    global status_fridge
    global messageCounter

    messageCounter += 1

    # skip messages on connect
    if messageCounter > 3:
        #print("Received message '" + str(message.payload) + "' on topic '"
        #    + message.topic + "' with QoS " + str(message.qos))

        if message.topic == "apliance/fridge":
            print("switch fridge")
            if status_fridge == "on":
                status_fridge = "off"
                GPIO.output(RELAIS_1_GPIO, GPIO.LOW)
                print('turn on')
            else:
                status_fridge = "on"
                GPIO.output(RELAIS_1_GPIO, GPIO.HIGH)
                print('turn off')
        elif message.topic == "apliance/mains":
            print("switch mains")
            if status_mains == "on":
                status_mains = "off"
                GPIO.output(RELAIS_MAIN_GPIO, GPIO.LOW)
            else:
                status_mains = "on"
                GPIO.output(RELAIS_MAIN_GPIO, GPIO.HIGH)
        elif message.topic == "apliance/LED":
            print("switch LED to " + str(message.payload))
            rgb = str(message.payload)
            rgb = rgb[3:]
            rgb = rgb[:-2]
            rgb = rgb.replace(" ", "")
            rgb = rgb.split(",")

            pixels.fill((int(rgb[0]),int(rgb[2]),int(rgb[1])))
            pixels.show()



        print('********************************')
        print('')
        print('')

# MQTT Client
client = paho.Client("SnowballPI") #create client object client1.on_publish = on_publish #assign function to callback client1.connect(broker,port) #establish connection client1.publish("house/bulb1","on")
paho.Client.connected_flag = False#create flag in class
client.on_message = on_message
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.loop_start() #start loop to process received messages
try:
    print("connecting to broker ",broker)
    client.connect(broker)#connect
except:
    print("could not connect")

while not client.connected_flag: #wait in loop
    print("Waiting for re-connect attempt")
    time.sleep(1)
print("subscribing ")
client.subscribe("apliance/mains")
client.subscribe("apliance/fridge")
client.subscribe("apliance/LED")
print('********************************')
print('')
print('')

# Main program loop
try:
    while True:
        print("Retrieving sensor data...")
        print('*********************************************')

        # read temp/humidity every 20 cycles as not needed so often
        if loopCount == 0:
            # temp/humidity
            sensorIndoor.trigger()
            time.sleep(.03) # Necessary on faster Raspberry Pi's
            indoor_humidity = ('{:3.2f}'.format(sensorIndoor.humidity() / 1.))
            indoor_temperature = ('{:3.2f}'.format(sensorIndoor.temperature() / 1.))
            print('Temp Inside:  {}C  Humidity Inside:  {}%'.format(indoor_temperature, indoor_humidity))
            client.publish("weather/inside/temperature",indoor_temperature)
            client.publish("weather/inside/humidity",indoor_humidity)
            #sensorIndoor.cancel()

            sensorOutdoor.trigger()
            time.sleep(.03) # Necessary on faster Raspberry Pi's
            outdoor_humidity = ('{:3.2f}'.format(sensorOutdoor.humidity() / 1.))
            outdoor_temperature = ('{:3.2f}'.format(sensorOutdoor.temperature() / 1.))
            print('Temp Outside: {}C  Humidity Outside: {}%'.format(outdoor_temperature, outdoor_humidity))
            client.publish("weather/outside/temperature",outdoor_temperature)
            client.publish("weather/outside/humidity",outdoor_humidity)
            #sensorOutdoor.cancel()

        # Voltage
        a0Value = adc.read_adc(0) # Read the ADC channel 0 value
        formatVoltage = remap_range(a0Value, 0, 32767, 0, 25)
        out_voltage = '{:.2f}'.format(formatVoltage/120.2*100)
        print("Voltage: " + str(out_voltage) + " V")
        client.publish("battery/voltage",out_voltage)
        client.publish("battery/voltage_bar",(formatVoltage/122*1000))

        # check for low Voltage and potentially shut everything down
        if float(out_voltage) < lowVoltageShutdownValue:
            # check 5 times
            lowVoltageCounter += 1
            print("Low Voltage detected. Will shut down if no change...")
            if lowVoltageCounter >= 5:
                print('')
                print('')
                print("Contious Low Voltage detected. Will shut down now.")
                print('*********************************************')
                print('')
                print('')
                # switch off mains relais
                GPIO.output(RELAIS_MAIN_GPIO, GPIO.LOW)
                #shutdown system
                os.system("shutdown now -h")
        else:
            # reset counter if next reading is above threshold
            lowVoltageCounter = 0


        # Current
        # get 10 readings and average
        a1Value = 0
        for x in range(1, 11):
            a1Value += adc.read_adc(3) #+ sensorZeroAdj;
        a1Value /= 10;

        currentADC3 = ((((a1Value * 0.125) - 2377) / 66))
        out_current = '{:.2f}'.format(currentADC3)
        print("Current: " + str(out_current) + " A")
        client.publish("battery/current",out_current)

        # Power
        power = '{:.2f}'.format(formatVoltage * currentADC3)
        print("Power:   " + str(power) + " W")
        client.publish("battery/power", power)

        print('*********************************************')
        print('')
        print('')
        loopCount += 1
        if loopCount == 20:
            loopCount = 0;
        time.sleep(30)


# Scavenging work after the end of the program
except KeyboardInterrupt:
    pi.stop()
    client.disconnect() #disconnect
    client.loop_stop() #stop loop
    print('Script end!')
