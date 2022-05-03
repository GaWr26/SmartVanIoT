import RPi.GPIO as GPIO
import time
import datetime
import board
import serial
import logging
import re
import paho.mqtt.client as mqtt
import json
import urllib.request
from threading import Thread
from subprocess import call
from gps import UpdateGPS
from cloud import UpdateThingspeak
from sensors import *
from led import LED
from sms import SMS
from simmodule import SIMMODULE

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
cloud_update_delay = 300
sim_update_counter = 0

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
GPIO.output(RELAIS_1_GPIO, 0)
GPIO.setup(RELAIS_2_GPIO, GPIO.OUT)
GPIO.output(RELAIS_2_GPIO, 1)
GPIO.setup(RELAIS_3_GPIO, GPIO.OUT)
GPIO.output(RELAIS_3_GPIO, 0)
GPIO.setup(RELAIS_4_GPIO, GPIO.OUT)
GPIO.output(RELAIS_4_GPIO, 0)

# Taster
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


text_message = ""

active_clients = 1


def on_connect(client, userdata, flags, rc):
    print("Connected to Broker")
    print('')

    client.subscribe("$SYS/broker/clients/active")
    client.subscribe("snowball/light/set_color")
    client.subscribe("snowball/light/dimmer")
    client.subscribe("snowball/light/dimmer")
    client.subscribe("snowball/light/effect/1")
    client.subscribe("snowball/light/effect/2")
    client.subscribe("snowball/light/effect/3")
    client.subscribe("snowball/light/effect/4")
    client.subscribe("snowball/switch/1")
    client.subscribe("snowball/switch/2")
    client.subscribe("snowball/switch/3")
    client.subscribe("snowball/switch/4")
    client.subscribe("snowball/light/effect/rainbow")
    client.subscribe("snowball/switch/calibrate_capacity")

    client.on_connect = ""

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global dimmer
    global active_clients
    global battery_capacity_ahs
    global amphour_start_time
    global amphours_charge_total
    global amphours_draw_total
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

    elif msg.topic == 'snowball/light/effect/1':
        ledcontrol.toggleAnimation(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/light/effect/2':
        ledcontrol.toggleAnimation(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/light/effect/3':
        ledcontrol.toggleAnimation(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/light/effect/4':
        ledcontrol.toggleAnimation(msg.topic[-1], msg.payload)
    elif msg.topic == 'snowball/switch/calibrate_capacity':
        print("Battery now set to fully charged")
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
        GPIO.output(eval("RELAIS_"+switch+"_GPIO"), GPIO.LOW) # an
        if switch == "2":
            print("LEDS on")
            client.publish("snowball/light/dimmer", dimmer_value)
            #time.sleep(3)
            client.publish("snowball/light/dimmer", dimmer_value)
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
print("   " + str(datetime.datetime.now()))
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

class Manager(object):
    global sensordata
    global phone_number
    global text_message

    def new_simmodule_thread(self):
        return SIMMODULE(parent=self)
    def on_gps_data(self, thread, data):
        #print("got sim data")
        try:
            sensordata["lat"] = data["lat"]
            sensordata["long"] = data["long"]
            sensordata["alt"] = data["alt"]
            sensordata["speed"] = data["speed"]
            try:
                client.publish("snowball/position/alt", sensordata["alt"])
                client.publish("snowball/position/speed", sensordata["speed"])
                client.publish("snowball/position/lat_long", str(sensordata["lat"]) + "," + str(sensordata["long"]) + ",0.0000000,0.0")
            except:
                print("retry mqtt publish")
        except:
            print("not able to parse GPS Data")

    def new_cloud_update_thread(self):
        return UpdateThingspeak(sensordata, parent=self)
    def on_cloud_update_thread_finished(self, thread):
        global modem_busy
        modem_busy = False

    def new_led_thread(self):
        return LED(parent=self)

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
        try:
            client.publish("snowball/sensor/remaining_ahs",sensordata["remaining_ahs"])
            client.publish("snowball/sensor/time_remaining",sensordata["time_remaining"])
        except:
            print("AHS not ready")

        #print("Sensor Data sent to MQTT broker")


mgr = Manager()

ledcontrol = mgr.new_led_thread()
ledcontrol.start()

simmodule = mgr.new_simmodule_thread()
simmodule.start()

sensor_thread = mgr.new_sensor_thread()
sensor_thread.start()

def is_online(host='http://google.com'):
    try:
        urllib.request.urlopen(host) #Python 3.x
        return True
    except:
        return False

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
        simmodule.sendsms(text_message)

    if lastLowDetection > 0 and time.time() - lastLowDetection >= warning_time + power_off_time:
        print("Shit is going down!")
        text_message = 'ACHTUNG! VanBox schaltet jetzt ab um die Batterie zu retten.'
        simmodule.sendsms(text_message)
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
            if skip_first >= 10:
                if battery_capacity_ahs == 0:
                    if sensordata["draw_voltage"] >= 12.7:
                        currentvoltage = 12.7
                    else:
                        currentvoltage = sensordata["draw_voltage"]
                    battery_capacity_ahs = battery_set_ahs / 100 * ((currentvoltage - 12) / 0.7 * 100)
                    print("Errechnete Batteriekapazitaet: " +  str(battery_capacity_ahs))

                if amphour_start_time == 0:
                    amphour_start_time = time.time()


                # subtract draw
                amphours_draw_total = amphours_draw_total + (sensordata["draw_current"] * (time.time() - lastSensorUpdate)/3600*-1)
                amphours_draw_total = amphours_draw_total
                amphours_charge_total = amphours_charge_total + (sensordata["charge_current"] * (time.time() - lastSensorUpdate)/3600)
                amphours_charge_total = amphours_charge_total
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

            skip_first = skip_first + 1

        #print (json.dumps(sensordata, indent=2))

        sensor_thread.getData()
        lastSensorUpdate = time.time()


    if lastCloudUpdate == 0 or time.time() - lastCloudUpdate >= cloud_update_delay:
        if skip_first >= 11:
            if is_online():
                #print("Updating Cloud via WIFI")
                cloudupdate = mgr.new_cloud_update_thread()
                cloudupdate.start()
                sim_update_counter = 0
            else:
                if(sim_update_counter == 0):
                    simmodule.update_cloud(sensordata)
                    #print("Updating Cloud via SIM")
                    sim_update_counter = sim_update_counter + 1
                    if(sim_update_counter == 4):
                        sim_update_counter = 0
                else:
                    print("Skipping Cloud update via SIM")

            lastCloudUpdate = time.time()
