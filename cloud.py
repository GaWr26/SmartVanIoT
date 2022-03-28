import urllib.request
import time

global sensordata
sensordata = {}
sensordata["humidity_inside"] = 0
sensordata["temp_inside"] = 0


class UpdateThingspeak:
    def __init__(self, data):
        global sensordata
        sensordata = data
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global sensordata
        print('********************************')
        print('     Starting Cloud Update')
        print('********************************')
        print('')
        print('')

        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1=" + str(sensordata["humid_inside"]))
        time.sleep(20)
        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field2=" + str(sensordata["temp_inside"]))
        time.sleep(20)
        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field3=" + str(sensordata["draw_voltage"]))
        time.sleep(20)
        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field4=" + str(sensordata["charge_current"]))
        time.sleep(20)
        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field5=" + str(sensordata["draw_current"]))
        time.sleep(20)
        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field6=" + str(sensordata["lat"]))
        time.sleep(20)
        urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field7=" + str(sensordata["long"]))

        print('********************************')
        print('     Cloud Update finished')
        print('********************************')
        print('')
        print('')
        self.terminate()
