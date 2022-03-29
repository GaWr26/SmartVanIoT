import urllib.request
import time
import serial
import threading

global sensordata
sensordata = {}
sensordata["humidity_inside"] = 0
sensordata["temp_inside"] = 0

#SIM & GPS Module
sim_serial = serial.Serial('/dev/ttyUSB3',115200)


class UpdateThingspeak(threading.Thread):
    def __init__(self, data, parent=None):
        global sensordata
        sensordata = data
        self.parent = parent
        super(UpdateThingspeak, self).__init__()

    def target_with_callback(self):
        self.method()
        if self.callback is not None:
            self.callback(*self.callback_args)

    def terminate(self):
        self._running = False

    def is_online(self, host='http://google.com'):
        try:
            urllib.request.urlopen(host) #Python 3.x
            return True
        except:
            return False

    def send_at(self, command,back,timeout):
        rec_buff = ''
        sim_serial.write((command+'\r\n').encode())
        time.sleep(timeout)
        if sim_serial.inWaiting():
            time.sleep(0.01 )
            rec_buff = sim_serial.read(sim_serial.inWaiting())
            if rec_buff != '':
                if back not in rec_buff.decode():
                    #print(command + ' ERROR')
                    print(command + ' back:\t' + rec_buff.decode())
                    return 0
                else:
                    return 1
            else:
                print('Modem is not ready')
                return 0

    def run(self):
        global sensordata
        if self.is_online():
            print('')
            print('********************************')
            print(' Starting Cloud Update via Wifi')
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

            print('')
            print('********************************')
            print('  Wifi Cloud Update finished')
            print('********************************')
            print('')
            print('')
        else:
            print('')
            print('********************************')
            print(' Starting Cloud Update via SIM')
            print('********************************')
            print('')
            print('')
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1=' + str(sensordata["humid_inside"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field2=' + str(sensordata["temp_inside"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field3=' + str(sensordata["draw_voltage"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field4=' + str(sensordata["charge_current"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field5=' + str(sensordata["draw_current"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field6=' + str(sensordata["lat"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at('AT+CSQ','OK',1)
            self.send_at('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at('AT+HTTPINIT','OK',1)
            self.send_at('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field7=' + str(sensordata["long"]) + '"','OK',1)
            self.send_at('AT+HTTPACTION=0 ','OK',1)
            print('********************************')
            print('   SIM Cloud Update finished')
            print('********************************')
            print('')
            print('')
        sim_serial.close()
        self.parent and self.parent.on_cloud_update_thread_finished(self)
        self.terminate()
