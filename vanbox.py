import RPi.GPIO as GPIO

import time
import adafruit_ahtx0
import Adafruit_DHT
import board
import logging
import board

import re
import paho.mqtt.client as mqtt


import json

from ina219 import INA219
from ina219 import DeviceRangeError
from pigpio_dht import DHT11, DHT22
from mpu6050 import mpu6050
from threading import Thread

from gps import UpdateGPS
from cloud import UpdateThingspeak
from sensors import *
from led import LedControl

sensordata = {}

sensordata["lat"] = 0
sensordata["long"] = 0

# Delays and times
lastCloudUpdate = 0
cloud_update_delay = 3600
lastGPSUpdate = 0
gps_update_delay = 600

# Sensors
indoorTempSensor = adafruit_ahtx0.AHTx0(board.I2C())
mpu = mpu6050(0x68)

SHUNT_OHMS = 0.0015
MAX_EXPECTED_AMPS = 50

ina_draw = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x44)
ina_charge = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
ina_draw.configure(ina_draw.RANGE_16V, ina_draw.GAIN_2_80MV)
ina_charge.configure(ina_charge.RANGE_16V, ina_charge.GAIN_2_80MV)


# LED
ledcontrol = LedControl()


# _GPIO
GPIO.setmode(GPIO.BCM)

RELAIS_1_GPIO = 5
RELAIS_2_GPIO = 6
RELAIS_4_GPIO = 19
RELAIS_3_GPIO = 13
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_2_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_3_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_4_GPIO, GPIO.OUT)

# Taster
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

def get_smoothed_values(n_samples=10):
    """
    Get smoothed values from the sensor by sampling
    the sensor `n_samples` times and returning the mean.
    """
    result = {}
    for _ in range(n_samples):
        data = mpu.get_accel_data()

        for k in data.keys():
            # Add on value / n_samples (to generate an average)
            # with default of 0 for first loop.
            result[k] = result.get(k, 0) + (data[k] / n_samples)
    sensordata["acc_x"] = str((result['x']-0.3)*100+50)
    sensordata["acc_y"] = str((result['y']+0.35)*100+50)
    #return result

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected to Broker with result code "+str(rc))
    print('')
    print('')

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
        ledcontrol.setLightColor(str(msg.payload)[3:9])
    elif msg.topic == 'snowball/light/dimmer':
        ledcontrol.setLightDimmer(str(msg.payload))
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

def toggle_LEDs():
    if GPIO.input(RELAIS_2_GPIO) == GPIO.HIGH:
        GPIO.output(RELAIS_2_GPIO, GPIO.LOW) # an
        client.publish("snowball/switch/2","off")
    else:
        GPIO.output(RELAIS_2_GPIO, GPIO.HIGH) # an
        client.publish("snowball/switch/2","on")


print('')
print('********************************')
print('     VANBOX is starting up....')
print('********************************')
print('')
print('[press ctrl+c to end the Program]')
print('')
print('')


print("Connecting to MQTT Broker...")
client = mqtt.Client("Pi") #create new instance
client.connect("127.0.0.1", 1883, 60)
client.on_connect = on_connect
client.on_message = on_message
client.loop_start()



while True:
    if GPIO.input(20) == GPIO.HIGH:
        toggle_LEDs()

    sensordata["draw_voltage"] = ina_draw.voltage()
    try:
        sensordata["draw_current"] = ina_draw.current()/1000
        sensordata["draw_power"] = (ina_draw.power()/1000) * -1
    except DeviceRangeError as e:
        print("Draw Current overflow")

    sensordata["charge_voltage"] = ina_charge.voltage()
    try:
        sensordata["charge_current"] = ina_charge.current()/1000
        sensordata["charge_power"] = ina_charge.power()/1000
    except DeviceRangeError as e:
        print("Charge Current overflow")


    sensordata["temp_inside"] = indoorTempSensor.temperature
    sensordata["humid_inside"] = indoorTempSensor.relative_humidity

    try:
        humidity, temperature = Adafruit_DHT.read_retry(22, 12)
        sensordata["temp_outside"] = temperature
        sensordata["humid_outside"] = humidity
    except DeviceRangeError as e:
        print("Error getting outside temperature")

    get_smoothed_values()

    # SEND IT - to MQTT
    client.publish("snowball/sensor/temp_inside",sensordata["temp_inside"])
    client.publish("snowball/sensor/humidity_inside",sensordata["humid_inside"])
    client.publish("snowball/sensor/temp_outside",sensordata["temp_outside"])
    client.publish("snowball/sensor/humidity_outside",sensordata["humid_outside"])
    client.publish("snowball/sensor/battery_draw_voltage", sensordata["draw_voltage"])
    client.publish("snowball/sensor/battery_draw_current", sensordata["draw_current"])
    client.publish("snowball/sensor/battery_draw_power", sensordata["draw_power"])
    client.publish("snowball/sensor/battery_charge_voltage", sensordata["charge_voltage"])
    client.publish("snowball/sensor/battery_charge_current", sensordata["charge_current"])
    client.publish("snowball/sensor/battery_charge_power", sensordata["charge_power"])
    client.publish("snowball/sensor/accelerometer_x",sensordata["acc_x"])
    client.publish("snowball/sensor/accelerometer_y",sensordata["acc_y"])
    client.publish("snowball/position/lat_long", str(sensordata["lat"]) + "," + str(sensordata["long"]) + ",0.0000000,0.0")

    #print (json.dumps(sensordata, indent=2))


    # Update thingspeak?
    if lastCloudUpdate == 0 or time.time() - lastCloudUpdate >= cloud_update_delay:
        UpdateCloudClass = UpdateThingspeak(sensordata)
        UpdateCloudThread = Thread(target=UpdateCloudClass.run)
        UpdateCloudThread.start()
        lastCloudUpdate = time.time()

    # Update GPS Position?
    if lastGPSUpdate == 0 or time.time() - lastGPSUpdate >= gps_update_delay:
        UpdateGPSClass = UpdateGPS()
        UpdateGPSThread = Thread(target=UpdateGPSClass.run)
        UpdateGPSThread.start()
        lastGPSUpdate = time.time()




    time.sleep(1)
