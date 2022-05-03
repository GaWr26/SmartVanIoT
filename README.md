# SmartVanIoT - VanBox

The story starts three years ago when I bought an almost dead VW T3. I re-built it completely and got to a point where I started planning the electric setup. I pretty soon got very frustrated by the amount of standalone components e.g Battery meter and their cost. Having a background in this kind of stuff I started to build my own solution.The idea is to have a system, that you connect to your second battery, and all other appliances just get connected to the box. I built the first version and had it in my Van last year. I got a lot of interest from people when I showed them what it does. This made me think to do a second iteration and plan it in way that would scale.

What I now have is v2 and I would like to share what it does:

It's designed to be simple and flexible. You connect the battery and all power inputs e.g. Solar charge controller, Mains Charger etc, to the inputs on the left.On the right you have the connections for the appliances on the top, and sensors and buttons etc. at the bottom.

One key component to the system is the SIM Module which enables cloud updates and gps positioning. So basically my Van is an IoT device that updates it's sensor and position values to the cloud so I always know where it is and e.g how the battery is doing.

I connected temperature and humidity sensors inside and outside the van.There's a gyro for helping to park level so I can sleep well :) It measures voltage, draw and power on the appliance side and on the inputs. If the battery capacity drops below a certain threshold, the system notifies me and if it drops even further it removes all appliances from power and shuts itself down to protect the battery.

It's able to connect and control digital LEDs

And of course there's an App. Here you can switch the appliances, control the light, check the position and relevant sensor data. My favourite feature is that it shows you how much battery time you have left with the current load.

If my phone is not present, the system switches to away mode and I get notified about movement and vibrations.

The brain currently is a raspberry pi zero using mqtt for communication and running a python app as the server.
