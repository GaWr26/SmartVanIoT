import serial
import time


#SIM & GPS Module
sim_serial = serial.Serial('/dev/ttyUSB3',115200)


class UpdateGPS:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def send_at(self,command,back,timeout):

        rec_buff = ''
        sim_serial.write((command+'\r\n').encode())
        time.sleep(timeout)
        if sim_serial.inWaiting():
            time.sleep(0.01 )
            rec_buff = sim_serial.read(sim_serial.inWaiting())
            if rec_buff != '':
                if back not in rec_buff.decode():
                    #print(command + ' ERROR')
                    #print(command + ' back:\t' + rec_buff.decode())
                    return 0
                else:
                    last_position = rec_buff.decode()
                    try:
                        sensordata["lat"] = float(last_position[25:36])/100
                        sendordata["long"] = float(last_position[39:51])/100
                        print('********************************')
                        print('         GPS Success')
                        print('********************************')
                        print('')
                        print('')
                        print(last_position)
                        print('')
                        print('')
                    except:
                        print('********************************')
                        print('         GPS Error')
                        print('********************************')
                        print('')
                        print('')
                    return 1
            else:
                print('Modem is not ready')
                return 0

    def run(self):
        print('********************************')
        print('      Starting GPS Update')
        print('********************************')
        rec_null = True
        answer = 0
        rec_buff = ''
        self.send_at('AT+CGPS=1,1','OK',1)
        time.sleep(2)
        while rec_null:
            answer = self.send_at('AT+CGPSINFO','+CGPSINFO: ',1)
            if 1 == answer:
                answer = 0
                if ',,,,,,' in rec_buff:
                    print('GPS is not ready')
                    rec_null = True
                    time.sleep(1)
                else:
                    #print("GPS OK")
                    rec_null = False
            else:
                #print('error %d'%answer)
                rec_buff = ''
                send_at('AT+CGPS=0','OK',1)
                return False
