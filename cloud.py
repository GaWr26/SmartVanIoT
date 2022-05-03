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

    def target_with_callback_cloud(self):
        self.method()
        if self.callback_cloud is not None:
            self.callback_cloud(*self.callback_cloud_args)

    def terminate(self):
        self._running = False

    def is_online(self, host='http://google.com'):
        try:
            urllib.request.urlopen(host) #Python 3.x
            return True
        except:
            return False

    def send_at_cloud(self, command,back_cloud,timeout):
        rec_buff_cloud = ''
        sim_serial.write((command+'\r\n').encode())
        time.sleep(timeout)
        if sim_serial.inWaiting():
            time.sleep(0.01 )
            rec_buff_cloud = sim_serial.read(sim_serial.inWaiting())
            if rec_buff_cloud != '':
                if back_cloud not in rec_buff_cloud.decode():
                    #print(command + ' ERROR')
                    print(command + ' back_cloud:\t' + rec_buff_cloud.decode())
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
            try:
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
            except:
                print('')
                print('********************************')
                print('  Wifi Cloud Update failed')
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

            if sim_serial.isOpen() == False:
                sim_serial.open()
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field1=' + str(sensordata["humid_inside"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field2=' + str(sensordata["temp_inside"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field3=' + str(sensordata["draw_voltage"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field4=' + str(sensordata["charge_current"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field5=' + str(sensordata["draw_current"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field6=' + str(sensordata["lat"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            time.sleep(20)
            self.send_at_cloud('AT+CSQ','OK',1)
            self.send_at_cloud('AT+CGSOCKCONT=1,"IP","CMNET"','OK',1)
            self.send_at_cloud('AT+HTTPINIT','OK',1)
            self.send_at_cloud('AT+HTTPPARA="URL","https://api.thingspeak.com/update?api_key=5BES7ZJMPH9KM58J&field7=' + str(sensordata["long"]) + '"','OK',1)
            self.send_at_cloud('AT+HTTPACTION=0 ','OK',1)
            print('********************************')
            print('   SIM Cloud Update finished')
            print('********************************')
            print('')
            print('')

        sim_serial.close()
        self.parent and self.parent.on_cloud_update_thread_finished(self)
        self.terminate()
