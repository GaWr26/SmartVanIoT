import serial
import time
import threading
import adafruit_ahtx0
import Adafruit_DHT
import board
from ina219 import INA219
from ina219 import DeviceRangeError
from mpu6050 import mpu6050

sensordata = {}

# Sensors
indoorTempSensor = adafruit_ahtx0.AHTx0(board.I2C())
mpu = mpu6050(0x68)

SHUNT_OHMS = 0.0015
MAX_EXPECTED_AMPS = 50

ina_draw = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x44)
ina_charge = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
ina_draw.configure(ina_draw.RANGE_16V, ina_draw.GAIN_2_80MV)
ina_charge.configure(ina_charge.RANGE_16V, ina_charge.GAIN_2_80MV)

class UpdateSensors(threading.Thread):
    def __init__(self, parent=None):
        self.parent = parent
        super(UpdateSensors, self).__init__()


    def target_with_callback(self):
        self.method()
        if self.callback is not None:
            self.callback(*self.callback_args)

    def terminate(self):
        self._running = False

    def get_smoothed_values(self, n_samples=10):
        """
        Get smoothed values from the sensor by sampling
        the sensor `n_samples` times and returning the mean.
        """
        global sensordata
        result = {}
        for _ in range(n_samples):
            data = mpu.get_accel_data()

            for k in data.keys():
                # Add on value / n_samples (to generate an average)
                # with default of 0 for first loop.
                result[k] = result.get(k, 0) + (data[k] / n_samples)
        sensordata["acc_x"] = str((result['x']-0.3)*100+50)
        sensordata["acc_y"] = str((result['y']+0.35)*100+50)
        #return result


    def run(self):
        global sensordata
        sensordata["draw_voltage"] = ina_draw.voltage()
        try:
            sensordata["draw_current"] = ina_draw.current()/1000
            sensordata["draw_power"] = (ina_draw.power()/1000) * -1
        except DeviceRangeError as e:
            print("Draw Current overflow")

        sensordata["charge_voltage"] = ina_charge.voltage()
        try:
            sensordata["charge_current"] = ina_charge.current()/1000-0.03
            sensordata["charge_power"] = ina_charge.power()/1000-0.6
        except DeviceRangeError as e:
            print("Charge Current overflow")


        sensordata["temp_inside"] = indoorTempSensor.temperature
        sensordata["humid_inside"] = indoorTempSensor.relative_humidity

        try:
            humidity, temperature = Adafruit_DHT.read_retry(22, 12)
            sensordata["temp_outside"] = temperature
            sensordata["humid_outside"] = humidity
        except DeviceRangeError as e:
            print("Error getting outside temperature")

        self.get_smoothed_values()

        self.parent and self.parent.on_sensor_thread_finished(self, sensordata)
        self.terminate()
