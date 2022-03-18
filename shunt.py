#!/usr/bin/env python

# Advanced - Manual Gain, High Resolution Example
import time
import board
import adafruit_dht
import urllib.request



import paho.mqtt.client as mqtt


from ina219 import INA219
from ina219 import DeviceRangeError

# Set the constants that were calculated
SHUNT_OHMS = 0.0015
MAX_EXPECTED_AMPS = 50

# Initial the dht device, with data pin connected to:
dhtDevice = adafruit_dht.DHT22(board.D18)


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
    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
	# Configure the object with the expected bus voltage
	# (either up to 16V or up to 32V with .RANGE_32V)
	# Also, configure the gain to be GAIN_2_80MW for the above example
    ina.configure(ina.RANGE_16V, ina.GAIN_2_80MV)

    global last_temp_inside
    global last_humid_inside
    global last_current
    global last_voltage
    global last_power
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
        #print("Shunt voltage: %.3f mV" % ina.shunt_voltage())
        print("")
        print("")
    except DeviceRangeError as e:
        print("Current overflow")

    try:
        # Print the values to the serial port
        temperature_c = dhtDevice.temperature
        humidity = dhtDevice.humidity
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


        #time.sleep(2.0)
    except Exception as error:
        dhtDevice.exit()
        raise error

    # SEND IT - to MQTT
    client.publish("snowball/sensor/temp_inside",last_temp_inside)
    client.publish("snowball/sensor/humidity_inside",last_humid_inside)

    client.publish("snowball/sensor/battery_voltage", last_voltage)
    client.publish("snowball/sensor/battery_current", last_current)
    client.publish("snowball/sensor/battery_power", last_power)

    done = time.time()
    elapsed = done - start
    print(elapsed)
    if elapsed >= 3600 or firstrun == True:
        firstrun = False
        start = time.time()
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
        else:
            print("send via SIM")




# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("snowball/weather/inside")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))


def connect(host='http://google.com'):
    try:
        urllib.request.urlopen(host) #Python 3.x
        return True
    except:
        return False



if __name__ == "__main__":
    client = mqtt.Client("Pi") #create new instance
    client.connect("127.0.0.1", 1883, 60)

    while True:
        read()
        time.sleep(2)
