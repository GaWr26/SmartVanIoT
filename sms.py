import serial
import time
import threading

#SIM & GPS Module
ser = serial.Serial('/dev/ttyUSB3',115200)

phone_number = ""
text_message = ""

class SMS(threading.Thread):
    def __init__(self, phone, text, parent=None):
        global phone_number
        global text_message
        phone_number = phone
        text_message = text
        self.parent = parent
        super(SMS, self).__init__()


    def target_with_callback(self):
        self.method()
        if self.callback is not None:
            self.callback(*self.callback_args)

    def terminate(self):
        self._running = False

    def send_at(self, command, back,timeout):
        rec_buff = ''
        ser.write((command+'\r\n').encode())
        time.sleep(timeout)
        if ser.inWaiting():
            time.sleep(0.01 )
            rec_buff = ser.read(ser.inWaiting())
        if back not in rec_buff.decode():
                print(command + ' ERROR')
                print(command + ' back:\t' + rec_buff.decode())
                return 0
        else:
            print(rec_buff.decode())
            return 1


    def run(self):
        print("Setting SMS mode...")
        self.send_at("AT+CMGF=1","OK",1)
        print("Sending Short Message")
        answer = self.send_at("AT+CMGS=\""+phone_number+"\"",">",2)
        if 1 == answer:
            ser.write(text_message.encode())
            ser.write(b'\x1A')
            answer = self.send_at('','OK',20)
            if 1 == answer:
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
            print('error%d'%answer)

        ser.close()
        self.parent and self.parent.on_sms_thread_finished(self)
        self.terminate()
