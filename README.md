<h1> Modification of an Emos E06016 replacement sensor to send both NMEA 0813 and NMEA 2000 datagrams </h1>

This repo describes the modification of an **[Emos E06016](https://www.amazon.de/EMOS-E06016-Drahtloser-Funkwindsensor-Wetterstation/dp/B08WXBMPBM)** replacement wind sensor, so it can send the NMEA 0813 information wirelessly or the NMEA 2000 via a CAN BUS.

<img width="500" alt="GUI" src="https://user-images.githubusercontent.com/16275519/147963059-f81f9713-367f-425f-8ff4-6b86bf068870.jpg">


The original sensor for the angle gets replaced by an **[AS5600](https://www.ebay.de/itm/164267218236)** magnetic orientation sensor to achieve a better resolution. 
The computing an wireless transmission of the NMEA Datagrams as well as the web interface is done with an **[ESP32](https://www.az-delivery.de/products/esp32-developmentboard)**.
The NMEA 2000 data can be send via an **[SN65HVD230](https://eckstein-shop.de/WaveshareSN65HVD230CANTransceiverBoard33V2CESDProtection)**.
The sensor is further equipped with sensors for both humidity and temperature. These can also be used by the ESP32 to measure and send these values. 

The modification reuses as many of the original components as possible, so that, theoretically, only a few parts are needed:
- 1 ESP32
- 1 AS5600
- 1 magnet
- 15 jumper wires
- 11 Luster Terminals

And, for NMEA 2000,
- 1 SN65HVD230
- 4 jumper wires

All the other components are taken from the original sensor. That reduces the overall costs to approximately 50€.

The firmware can either work standalone, where the sensor creates an access point that can be used to display the data on a tablet or phone without any further hardware, or be integrated in an existing setup.

This project is a continuation of the **[NMEA0182-Windsensor-project](https://github.com/jukolein/NMEA0183-Windsensor)** and the **[Ventus W132-Windsensor-project](https://github.com/jukolein/W132)**

The firmware can either connect the sensor to an existing network or create an access point.
It has a graphical display of the data and a settings page in English and German.

<img width="943" alt="gui_en" src="https://user-images.githubusercontent.com/16275519/148052165-69da0fca-3bb2-4132-a773-28239bb2534a.png">

<img width="947" alt="settings_1" src="https://user-images.githubusercontent.com/16275519/148051843-70e5f24d-4664-4037-894d-07f07472bcd0.png">
<img width="947" alt="settings_2" src="https://user-images.githubusercontent.com/16275519/148051594-b7336bb4-be0d-4a7a-aa5d-5dea53f66318.png">
