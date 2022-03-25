import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

RELAIS_1_GPIO = 5
RELAIS_2_GPIO = 6
RELAIS_4_GPIO = 19
RELAIS_3_GPIO = 13
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_2_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_3_GPIO, GPIO.OUT)
GPIO.setup(RELAIS_4_GPIO, GPIO.OUT)



GPIO.output(RELAIS_1_GPIO, GPIO.HIGH) # an
time.sleep(2)
GPIO.output(RELAIS_1_GPIO, GPIO.LOW) # an
time.sleep(2)

GPIO.output(RELAIS_2_GPIO, GPIO.HIGH) # an
time.sleep(2)
GPIO.output(RELAIS_2_GPIO, GPIO.LOW) # an
time.sleep(2)

GPIO.output(RELAIS_3_GPIO, GPIO.HIGH) # an
time.sleep(2)
GPIO.output(RELAIS_3_GPIO, GPIO.LOW) # an
time.sleep(2)

GPIO.output(RELAIS_4_GPIO, GPIO.HIGH) # an
time.sleep(2)
GPIO.output(RELAIS_4_GPIO, GPIO.LOW) # an
time.sleep(2)
