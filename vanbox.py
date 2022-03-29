import RPi.GPIO as GPIO
import time
import board
import logging
import re
import paho.mqtt.client as mqtt
import json
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
lastSensorUpdate = 0
sensor_update_delay = 1
lastToggle = 0

# LED
ledcontrol = LedControl()
dimmer = "b'100'"

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

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected to Broker with result code "+str(rc))
    print('')
    print('')

    client.subscribe("$SYS/broker/clients/active")
    client.subscribe("snowball/light/set_color")
    client.subscribe("snowball/light/dimmer")
    client.subscribe("snowball/light/dimmer")
    client.subscribe("snowball/switch/1")
    client.subscribe("snowball/switch/2")
    client.subscribe("snowball/switch/3")
    client.subscribe("snowball/switch/4")
    client.subscribe("snowball/light/effect/rainbow")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global dimmer
    print("Received Message: " + msg.topic+" "+str(msg.payload))
    if msg.topic == 'snowball/light/set_color':
        ledcontrol.setLightColor(str(msg.payload)[3:9])
    elif msg.topic == 'snowball/light/dimmer':
        dimmer = str(msg.payload)
        ledcontrol.setLightDimmer(dimmer)
    elif msg.topic == 'snowball/light/effect/rainbow':
        ledcontrol.rainbow_cycle(0.001)
    elif msg.topic == 'snowball/switch/1':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/2':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/3':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/4':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == '$SYS/broker/clients/active':
        dimmer_value = str(re.findall(r"'(.*?)'", dimmer)[0])
        client.publish("snowball/light/dimmer", dimmer_value)
        if GPIO.input(RELAIS_1_GPIO) == GPIO.LOW:
            client.publish("snowball/switch/1","off")
        else:
            client.publish("snowball/switch/1","on")

        if GPIO.input(RELAIS_2_GPIO) == GPIO.LOW:
            client.publish("snowball/switch/2","off")
        else:
            client.publish("snowball/switch/2","on")

        if GPIO.input(RELAIS_3_GPIO) == GPIO.LOW:
            client.publish("snowball/switch/3","off")
        else:
            client.publish("snowball/switch/3","on")

        if GPIO.input(RELAIS_4_GPIO) == GPIO.LOW:
            client.publish("snowball/switch/4","off")
        else:
            client.publish("snowball/switch/4","on")

def toggleSwitch(switch, value):
    new_value = re.findall(r"'(.*?)'", str(value))[0]
    if new_value == "on":
        print("switch " + switch + " on")
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.HIGH) # an
        if switch == "2":
            client.publish("snowball/light/dimmer", "100")
    else:
        print("switch " + switch + " off")
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.LOW) # an
        if switch == "2":
            client.publish("snowball/light/dimmer", "0")

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

def publish_to_mqtt():
    try:
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
        #print("Data sent to MQTT broker")
    except:
        print("Error posting to mqtt")

class Manager(object):
    def new_gps_thread(self):
        return UpdateGPS(parent=self)
    def on_gps_thread_finished(self, thread, data):
        sensordata["lat"] = data["lat"]
        sensordata["long"] = data["long"]

    def new_sensor_thread(self):
        return UpdateSensors(parent=self)
    def on_sensor_thread_finished(self, thread, data):
        #print (json.dumps(data, indent=2))
        sensordata["acc_x"] = data["acc_x"]
        sensordata["acc_y"] = data["acc_y"]
        sensordata["draw_voltage"] = data["draw_voltage"]
        sensordata["draw_current"] = data["draw_current"]
        sensordata["draw_power"] = data["draw_power"]
        sensordata["charge_voltage"] = data["charge_voltage"]
        sensordata["charge_current"] = data["charge_current"]
        sensordata["charge_power"] = data["charge_power"]
        sensordata["temp_inside"] = data["temp_inside"]
        sensordata["humid_inside"] = data["humid_inside"]
        sensordata["temp_outside"] = data["temp_outside"]
        sensordata["humid_outside"] = data["humid_outside"]

mgr = Manager()

while True:
    if GPIO.input(20) == GPIO.HIGH:
        if lastToggle == 0 or time.time() - lastToggle >= 1:
            lastToggle = time.time()
            toggle_LEDs()

    if lastSensorUpdate == 0 or time.time() - lastSensorUpdate >= sensor_update_delay:
        sensor_thread = mgr.new_sensor_thread()
        sensor_thread.start()
        publish_to_mqtt()
        lastSensorUpdate = time.time()

    # Update thingspeak?
    if lastCloudUpdate == 0 or time.time() - lastCloudUpdate >= cloud_update_delay:
        UpdateCloudClass = UpdateThingspeak(sensordata)
        UpdateCloudThread = Thread(target=UpdateCloudClass.run)
        UpdateCloudThread.start()
        lastCloudUpdate = time.time()

    # Update GPS Position?
    if lastGPSUpdate == 0 or time.time() - lastGPSUpdate >= gps_update_delay:
        gps_thread = mgr.new_gps_thread()
        gps_thread.start()
        lastGPSUpdate = time.time()
