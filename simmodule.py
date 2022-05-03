import serial
import time
import threading
import re
from pynmeagps import NMEAReader


#SIM & GPS Module
sim_serial = serial.Serial('/dev/ttyUSB3',115200)
sim_serial.flushInput()

sensordata = {}
phone_number = "015203593625"
pause_gps = False

class SIMMODULE(threading.Thread):
    def __init__(self, parent=None):
        self.parent = parent
        super(SIMMODULE, self).__init__()


    def target_with_callback(self):
        self.method()
        if self.callback is not None:
            self.callback(*self.callback_args)

    def terminate(self):
        self._running = False

    def send_at_sms(self, command, back_sms, timeout):
        rec_buff_sms = ''
        sim_serial.write((command+'\r\n').encode())
        time.sleep(timeout)
        if sim_serial.inWaiting():
            time.sleep(0.01 )
            rec_buff_sms = sim_serial.read(sim_serial.inWaiting())
        if back_sms not in rec_buff_sms.decode():
                print(command + ' ERROR')
                print(command + ' back:\t' + rec_buff_sms.decode())
                return 0
        else:
            print(rec_buff_sms.decode())
            return 1

    def sendsms(self, text):
        global pause_gps
        print("Setting SMS mode...")
        pause_gps = True
        time.sleep(1)
        sim_serial.flushInput()
        self.send_at_sms("AT+CMGF=1","OK",1)
        print("Sending Short Message: " + text)
        answer_sms = self.send_at_sms("AT+CMGS=\""+phone_number+"\"",">",2)
        if 1 == answer_sms:
            sim_serial.write(text.encode())
            sim_serial.write(b'\x1A')
            answer_sms = self.send_at_sms('','OK',20)
            if 1 == answer_sms:
                print('')
                print('************************************')
                print('      SMS sent successfully')
                print('*************************************')
                print('')
            else:
                print('')
                print('************************************')
                print('       Error sending SMS')
                print('*************************************')
                print('')

        else:
            print('error%d'%answer_sms)
        time.sleep(1)
        pause_gps = False

    def send_at_cloud(self, command_cloud, back_cloud,timeout):
        rec_buff_cloud = ''
        sim_serial.write((command_cloud+'\r\n').encode())
        time.sleep(timeout)
        if sim_serial.inWaiting():
            time.sleep(0.01 )
            rec_buff_cloud = sim_serial.read(sim_serial.inWaiting())
            if rec_buff_cloud != '':
                if back_cloud not in rec_buff_cloud.decode():
                    #print(command + ' ERROR')
                    print(command_cloud + ' back_cloud:\t' + rec_buff_cloud.decode())
                    return 0
                else:
                    return 1
            else:
                print('Modem is not ready')
                return 0


    def update_cloud(self, sensordata):
        global pause_gps
        pause_gps = True
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
        time.sleep(1)
        pause_gps = False

    def send_at_gps(self,command,back,timeout):
        global sensordata
        rec_buff = ''
        sim_serial.write((command+'\r\n').encode())
        time.sleep(timeout)
        #print("Errorcounter: " + str(error_counter))
        if sim_serial.inWaiting():
            time.sleep(0.01 )
            rec_buff = sim_serial.read(sim_serial.inWaiting())
        if rec_buff != '':
            if back not in rec_buff.decode():
                #print(command + ' ERROR')
                #print(command + ' back:\t' + rec_buff.decode())
                return 0
            else:
                #print(rec_buff.decode())
                last_position = rec_buff.decode()
                #data = re.findall(r",(.*),", str(last_position))
                gps_list = last_position.split(",")
                #msg = pynmea2.parse(last_position)
                #msg = NMEAReader.parse(last_position)
                #print("MSG: " + str(msg))
                try:
                    #sensordata["lat"] = float(last_position[25:36])#/100
                    #sensordata["long"] = float(last_position[39:51])#/100
                    lat = float(last_position[25:36])#/100
                    long = float(last_position[39:51])#/100

                    sensordata["alt"] = gps_list[6]
                    sensordata["speed"] = float(gps_list[7])*1.609

                    #So as latitude is in format DDMM.MMMMM
                    DDLat = int(float(lat)/100)
                    MMLat = float(lat) - DDLat * 100
                    sensordata["lat"] = DDLat + MMLat/60

                    DDLong = int(float(long)/100)
                    MMLong = float(long) - DDLong * 100
                    sensordata["long"] = DDLong + MMLong/60
                    print('')
                    print('********************************')
                    print('         GPS Success')
                    print("    lat: " + str(sensordata["lat"]))
                    print("   long: " + str(sensordata["long"]))
                    print("    alt: " + str(sensordata["alt"]))
                    print("  speed: " + str(sensordata["speed"]))
                    print('********************************')
                    print('')
                    print('')
                    return 1
                except:
                    print('')
                    print('********************************')
                    print('         GPS Error')
                    print('********************************')
                    print('')
                    print('')
                    return 0

        else:
            #print('GPS is not ready')
            return 0

    def run(self):
        global sensordata
        global pause_gps
        print('')
        print('********************************')
        print('      Starting SIM Module')
        print('********************************')
        print('')


        print('')
        print('********************************')
        print('    Starting GPS Positioning')
        print('********************************')
        print('')



        if sim_serial.isOpen() == False:
            sim_serial.open()
        rec_null = True
        answer = 0
        rec_buff = ''
        self.send_at_gps('AT+CGPS=1,1','OK',1)
        time.sleep(2)
        while rec_null:
            if pause_gps == False:
                answer = self.send_at_gps('AT+CGPSINFO','+CGPSINFO: ',1)
                if 1 == answer:
                    answer = 0
                    if ',,,,,,' in rec_buff:
                        print('GPS is not ready')
                        rec_null = True
                        time.sleep(1)
                    else:
                        #print("GPS OK")
                        self.parent and self.parent.on_sim_data(self, sensordata)
                        #rec_null = False
                time.sleep(0.5)

        #sim_serial.close()

        #self.terminate()
