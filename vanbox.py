import RPi.GPIO as GPIO
import time
import board
import serial
import logging
import re
import paho.mqtt.client as mqtt
import json
from threading import Thread
from subprocess import call
from gps import UpdateGPS
from cloud import UpdateThingspeak
from sensors import *
from led import LED
from sms import SMS

system_run = True

sensordata = {}
sensordata["lat"] = 0
sensordata["long"] = 0

current_poweroff = 12.00
lastLowDetection = 0
warning_time = 120
power_off_time = 120
warned = False


amphours_draw_total = 0.0
amphours_draw_current = 0.0
amphours_charge_total = 0.0
amphours_charge_current = 0.0
battery_capacity_ahs = 0
battery_set_ahs = 45
skip_first = 0

modem_busy = False

# Delays and times
lastCloudUpdate = 0
cloud_update_delay = 3600
lastGPSUpdate = 0
gps_update_delay = 600
lastSensorUpdate = 0
sensor_update_delay = 1
lastToggle = 0
amphour_start_time = 0

# LED
dimmer = "b'100'"

# _GPIO
GPIO.setmode(GPIO.BCM)
RELAIS_1_GPIO = 5
RELAIS_2_GPIO = 6
RELAIS_4_GPIO = 19
RELAIS_3_GPIO = 13
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT)
GPIO.output(RELAIS_1_GPIO, 1)
GPIO.setup(RELAIS_2_GPIO, GPIO.OUT)
GPIO.output(RELAIS_2_GPIO, 1)
GPIO.setup(RELAIS_3_GPIO, GPIO.OUT)
GPIO.output(RELAIS_3_GPIO, 1)
GPIO.setup(RELAIS_4_GPIO, GPIO.OUT)
GPIO.output(RELAIS_4_GPIO, 1)

# Taster
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

phone_number = "015203593625"
text_message = ""

active_clients = 1


def on_connect(client, userdata, flags, rc):
    print("Connected to Broker")
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
    client.subscribe("snowball/sensor/calibrate_battery")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global dimmer
    global active_clients
    dimmer_value = str(re.findall(r"'(.*?)'", dimmer)[0])
    print("Received Message: " + msg.topic+" "+str(msg.payload))
    if msg.topic == 'snowball/light/set_color':
        ledcontrol.setLightColor(str(msg.payload)[3:9])
    elif msg.topic == 'snowball/light/dimmer':
        dimmer = str(msg.payload)
        ledcontrol.setLightDimmer(dimmer)
    elif msg.topic == 'snowball/light/effect/rainbow':
        ledcontrol.rainbow_cycle(0.01)
    elif msg.topic == 'snowball/switch/1':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/2':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/3':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/4':
        toggleSwitch(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/sensor/calibrate_battery':
        battery_capacity_ahs = battery_set_ahs
        amphour_start_time = 0
        amphours_charge_total = 0.0
        amphours_draw_total = 0.0
    elif msg.topic == '$SYS/broker/clients/active':
        # check if # clients has increased.
        active_client_count = active_clients
        active_clients = int(re.findall(r"'(.*?)'", str(msg.payload))[0])
        if active_clients > active_client_count:
            # send values to initialize app UI
            if GPIO.input(RELAIS_1_GPIO) == GPIO.HIGH:
                client.publish("snowball/switch/1","off")
            else:
                client.publish("snowball/switch/1","on")

            if GPIO.input(RELAIS_2_GPIO) == GPIO.HIGH:
                client.publish("snowball/switch/2","off")
            else:
                client.publish("snowball/switch/2","on")
                client.publish("snowball/light/dimmer", dimmer_value)

            if GPIO.input(RELAIS_3_GPIO) == GPIO.HIGH:
                client.publish("snowball/switch/3","off")
            else:
                client.publish("snowball/switch/3","on")

            if GPIO.input(RELAIS_4_GPIO) == GPIO.HIGH:
                client.publish("snowball/switch/4","off")
            else:
                client.publish("snowball/switch/4","on")

def toggleSwitch(switch, value):
    global dimmer
    dimmer_value = str(re.findall(r"'(.*?)'", dimmer)[0])
    new_value = re.findall(r"'(.*?)'", str(value))[0]
    if new_value == "on":
        if switch == "2":
            client.publish("snowball/light/dimmer", dimmer_value)
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.LOW) # an

    else:
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.HIGH) # an

def toggle_LEDs():
    if GPIO.input(RELAIS_2_GPIO) == GPIO.LOW:
        GPIO.output(RELAIS_2_GPIO, GPIO.HIGH) # an
        client.publish("snowball/switch/2","off")
    else:
        GPIO.output(RELAIS_2_GPIO, GPIO.LOW) # an
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
        client.publish("snowball/sensor/remaining_ahs",sensordata["remaining_ahs"])
        client.publish("snowball/sensor/time_remaining",sensordata["time_remaining"])
        client.publish("snowball/position/lat_long", str(sensordata["lat"]) + "," + str(sensordata["long"]) + ",0.0000000,0.0")
        #print("Data sent to MQTT broker")
    except:
        print("retry mqtt publish")

class Manager(object):
    global sensordata
    global phone_number
    global text_message
    def new_gps_thread(self):
        return UpdateGPS(parent=self)
    def on_gps_thread_finished(self, thread, data):
        global modem_busy
        try:
            sensordata["lat"] = data["lat"]
            sensordata["long"] = data["long"]
        except:
            print("not able to parse GPS Data")
        modem_busy = False

    def new_sms_thread(self):
        return SMS(phone_number, text_message, parent=self)
    def on_sms_thread_finished(self, thread):
        global modem_busy
        modem_busy = False

    def new_led_thread(self):
        return LED(parent=self)

    def new_cloud_update_thread(self):
        return UpdateThingspeak(sensordata, parent=self)
    def on_cloud_update_thread_finished(self, thread):
        global modem_busy
        modem_busy = False

    def new_sensor_thread(self):
        return UpdateSensors(parent=self)
    def on_sensor_thread_finished(self, thread, data):
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

ledcontrol = mgr.new_led_thread()
ledcontrol.start()

while system_run:
    if GPIO.input(20) == GPIO.HIGH:
        if lastToggle == 0 or time.time() - lastToggle >= 1:
            lastToggle = time.time()
            toggle_LEDs()

    if lastLowDetection > 0 and time.time() - lastLowDetection >= warning_time and warned == False:
        warned = True
        print('')
        print('************************************')
        print('   Triggering Battery Warning')
        print(' If this stays, system will shutdown')
        print('*************************************')
        print('')
        text_message = 'ACHTUNG! Die Batterie ist bei '+ str(sensordata["draw_voltage"]) + "V. Wenn das noch weiterhin so bleibt schalte ich ab."
        modem_busy = True
        sms_thread = mgr.new_sms_thread()
        sms_thread.start()

    if lastLowDetection > 0 and time.time() - lastLowDetection >= warning_time + power_off_time:
        print("Shit is going down!")
        text_message = 'ACHTUNG! VanBox schaltet jetzt ab um die Batterie zu retten.'
        modem_busy = True
        sms_thread = mgr.new_sms_thread()
        sms_thread.start()
        time.sleep(60)
        call("sudo shutdown -h now", shell=True)
        system_run = False

    # every second except first run
    if lastSensorUpdate != 0 and time.time() - lastSensorUpdate >= sensor_update_delay:
        # check battery level against limit
        try:
            if float(sensordata["draw_voltage"]) < current_poweroff:
                if lastLowDetection == 0:
                    print('')
                    print('************************************')
                    print('   Battery Below Critical Limit')
                    print(' If this stays, warning will trigger')
                    print('*************************************')
                    print('')
                    lastLowDetection = time.time()
            else:
                lastLowDetection = 0
                warned = False
        except:
            print("")

    if lastSensorUpdate == 0 or time.time() - lastSensorUpdate >= sensor_update_delay:
        if lastSensorUpdate > 1:
            if skip_first >= 5:
                if battery_capacity_ahs == 0:
                    print(str(sensordata["draw_voltage"]))
                    battery_capacity_ahs = battery_set_ahs / 100 * ((sensordata["draw_voltage"] - 12) / 0.7 * 100)
                    print("Errechnete Batteriekapazitaet: " +  str(battery_capacity_ahs))

                if amphour_start_time == 0:
                    amphour_start_time = time.time()


                # subtract draw
                amphours_draw_total = amphours_draw_total + (sensordata["draw_current"] * (time.time() - lastSensorUpdate)/3600*-1)
                amphours_draw_total = round(amphours_draw_total,7)
                amphours_charge_total = amphours_charge_total + (sensordata["charge_current"] * (time.time() - lastSensorUpdate)/3600)
                amphours_charge_total = round(amphours_charge_total,7)
                #print('')
                #print("amphours_draw_total:" + str(amphours_draw_total))
                #print("amphours_charge_total:" + str(amphours_charge_total))
                #print('')
                sensordata["remaining_ahs"] = battery_capacity_ahs - amphours_draw_total + amphours_charge_total
                if sensordata["remaining_ahs"] > battery_set_ahs:
                    sensordata["remaining_ahs"] = battery_set_ahs
                sensordata["time_remaining"] = round(sensordata["remaining_ahs"]/(sensordata["draw_current"]*-1 - sensordata["charge_current"]))
                if sensordata["time_remaining"] < 0:
                    sensordata["time_remaining"] = "∞"

                if time.time()-amphour_start_time > 60:
                    amphour_start_time = 0
                    amphours_draw_total = 0
            skip_first = skip_first + 1
        #print (json.dumps(sensordata, indent=2))
        sensor_thread = mgr.new_sensor_thread()
        sensor_thread.start()
        publish_to_mqtt()
        lastSensorUpdate = time.time()

    # Update GPS Position?
    if lastGPSUpdate == 0 or time.time() - lastGPSUpdate >= gps_update_delay:
        if modem_busy == False:
            modem_busy = True
            gps_thread = mgr.new_gps_thread()
            gps_thread.start()
            lastGPSUpdate = time.time()

    # Update thingspeak?
    if lastCloudUpdate == 0 or time.time() - lastCloudUpdate >= cloud_update_delay:
        if modem_busy == False:
            modem_busy = True
            cloud_update_thread = mgr.new_cloud_update_thread()
            cloud_update_thread.start()
            lastCloudUpdate = time.time()
