# SmartVanIoT

Experimenting with Smart Sensors for my VW Camper Van.

Gathering Sensor-Data for:
- Battery voltage
- Battery current
- Temperature
- Humidity

![Setup](/images/setup.jpg.png)

Additionally controlling Relais for mains and all appliances.
As I have digital LEDs in the Van, I also added control for these.
If the voltage drops below a certain value, the mains relais shuts off and the raspberry pi shuts itself down.

All communication is based on the MQTT protocol and I use an instance of IoBroker on the pi for the MQTT Broker.
The brain is a Raspberry Pi Zero W which works great for this Application using 0.2 Amps in when idle.

To visualize everything I use an Android App called MQTT Dashboard where you can customize the Dashboard to your needs:
![Dashboard](/images/Dashboard_1.jpg)
![Dashboard](/images/Dashboard_2.jpg)
![Dashboard](/images/Dashboard_3.jpg)


Next up will be storing relevant data in an Influx Database and visualizing it via Grafana
