import Adafruit_DHT

humidity, temperature = Adafruit_DHT.read_retry(22, 4)
print(humidity, temperature)
