import urllib.request
import time
import datetime
import threading

global sensordata
sensordata = {}
sensordata["humidity_inside"] = 0
sensordata["temp_inside"] = 0



class UpdateThingspeak(threading.Thread):
    def __init__(self, data, parent=None):
        global sensordata
        sensordata = data
        self.parent = parent
        super(UpdateThingspeak, self).__init__()

    def target_with_callback_cloud(self):
        self.method()
        if self.callback_cloud is not None:
            self.callback_cloud(*self.callback_cloud_args)

    def terminate(self):
        self._running = False


    def run(self):
        global sensordata
        print('')
        print('********************************')
        print(' Starting Cloud Update via Wifi')
        print("   " + str(datetime.datetime.now()))
        print('********************************')
        print('')
        print('')
        try:
            #urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1=" + str(sensordata["humid_inside"]))
            #time.sleep(20)
            #urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field2=" + str(sensordata["temp_inside"]))
            #time.sleep(20)
            #urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field3=" + str(sensordata["draw_voltage"]))
            #time.sleep(20)
            #urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field4=" + str(sensordata["charge_current"]))
            #time.sleep(20)
            #urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field5=" + str(sensordata["draw_current"]))
            #time.sleep(20)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field6=" + str(sensordata["lat"]))
            time.sleep(20)
            urllib.request.urlopen("https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field7=" + str(sensordata["long"]))
            print('')
            print('********************************')
            print('  Wifi Cloud Update finished')
            print("   " + str(datetime.datetime.now()))
            print('********************************')
            print('')
            print('')
        except:
            print('')
            print('********************************')
            print('  Wifi Cloud Update failed')
            print("   " + str(datetime.datetime.now()))
            print('********************************')
            print('')
            print('')

        self.parent and self.parent.on_cloud_update_thread_finished(self)
        self.terminate()
