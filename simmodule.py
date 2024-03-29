import serial
import serial.tools.list_ports as prtlst
import usb.core
import time
import datetime
import threading
import re
from pynmeagps import NMEAReader



usbport = 3
#SIM & GPS Module
#dev = usb.core.find(idVendor=0x1e0e, idProduct=0x9001)
#print(dev)
def connectModule():
    global usbport
    global sim_serial
    try:
        sim_serial = serial.Serial("/dev/ttyUSB" + str(usbport),115200)
    except:
        usbport = usbport +1
        connectModule()

connectModule();

sim_serial.flushInput()

sensordata = {}
phone_number = "015203593625"
rec_buff = ""


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

    def send_at(self,command,back,timeout):
        global rec_buff
        rec_buff = ''
        try:
            sim_serial.write((command+'\r\n').encode())
            time.sleep(timeout)
            if sim_serial.inWaiting():
                time.sleep(0.01 )
                rec_buff = sim_serial.read(sim_serial.inWaiting())
            if rec_buff != "":
                #print("*******************************")
                #print(rec_buff.decode())
                #print("*******************************")
                if back not in rec_buff.decode():
                    #print(command + ' ERROR')
                    #print(command + ' back:\t' + rec_buff.decode())
                    return 0
                else:
                    return 1
            else:
                return 0
        except:
            print("Serial write failed")
            return 0



    def sendsms(self, text):
        print("Setting SMS mode...")
        sim_serial.flushInput()
        self.send_at("AT+CMGF=1","OK",1)
        print("Sending Short Message: " + text)
        answer_sms = self.send_at("AT+CMGS=\""+phone_number+"\"",">",1)
        if 1 == answer_sms:
            sim_serial.write(text.encode())
            sim_serial.write(b'\x1A')
            answer_sms = self.send_at('','OK',20)
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
        #time.sleep(1)

    def update_cloud(self, sensordata):
        global gps_active
        gps_active = False
        print('')
        print('********************************')
        print(' Starting Cloud Update via SIM')
        print("   " + str(datetime.datetime.now()))
        print('********************************')
        print('')
        print('')

        sim_serial.flushInput()
        time.sleep(1)
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
        self.send_at('AT+HTTPTERM','OK',1)
        print('********************************')
        print('   SIM Cloud Update finished')
        print("   " + str(datetime.datetime.now()))
        print('********************************')
        print('')
        print('')
        time.sleep(1)
        gps_active = True




    def run(self):
        global gps_active
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
        gps_active = True
        answer = 0
        self.send_at('AT+CGPS=1,1','OK',1)
        time.sleep(2)
        while True:
            if gps_active:
                answer = self.send_at('AT+CGPSINFO','+CGPSINFO: ',0.5)
                #print("gps poll...")
                if 1 == answer:
                    answer = 0
                    if ',,,,,,' in str(rec_buff):
                        print('No GPS Fix yet')
                        sensordata["alt"] = 0
                        sensordata["speed"] = 0
                        sensordata["lat"] = 0
                        sensordata["long"] = 0
                        time.sleep(1)
                    else:
                        try:
                            last_position = rec_buff.decode()
                            gps_list = last_position.split(",")
                            lat = float(last_position[25:36])#/100
                            long = float(last_position[39:51])#/100

                            sensordata["alt"] = gps_list[6]
                            sensordata["speed"] = float(gps_list[7])*1.852

                            #So as latitude is in format DDMM.MMMMM
                            DDLat = int(float(lat)/100)
                            MMLat = float(lat) - DDLat * 100
                            sensordata["lat"] = DDLat + MMLat/60

                            DDLong = int(float(long)/100)
                            MMLong = float(long) - DDLong * 100
                            sensordata["long"] = DDLong + MMLong/60
                            #print('')
                            #print('********************************')
                            #print('         GPS Success')
                            #print("    lat: " + str(sensordata["lat"]))
                            #print("   long: " + str(sensordata["long"]))
                            #print("    alt: " + str(sensordata["alt"]))
                            #print("  speed: " + str(sensordata["speed"]))
                            #print('********************************')
                            #print('')
                            #print('')
                        except:
                            sensordata["alt"] = 0
                            sensordata["speed"] = 0
                            print('')
                            print('********************************')
                            print('         GPS Error')
                            print('********************************')
                            print('')
                            print('')
                    self.parent and self.parent.on_gps_data(self, sensordata)
                #time.sleep(1)
