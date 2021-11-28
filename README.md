# Multilite
Oregon Scientific compatible Temperature, Humidity, UV and Luxmeter Sensor

This is an Oregon Scientific THGR810 / UVN8000 sensor emulator (Temperature,
  Humidity and UV) based on Joseph Shuy work: http://shuhy.com/esi/osv3_dock_sensor.htm

  Designed and modified by Daniele Sgroi, for OS Protocol 3.0, daniele.sgroi@gmail.com

  Using task scheduler from Anatoli Arkhipenko: https://github.com/arkhipenko/TaskScheduler

  Based on libraries written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution 

  information on these sensor was gleened from multiple places
  - the "Oregon Scientific RF Protocol Description" from May 2011 has data packet format 
    information http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf

  - sourceforge.net forums discussion of the CRC in the data packet

  - crc calculation code was taken from crc8.c published by Rajiv Chakravorty
    http://www.rajivchakravorty.com/source-code/uncertainty/multimedia-sim/html/crc8_8c-source.html

  Parts used include:
  - Arduino Pro Mini - ATMEGA 168P - 3.3V - 8 MHz (voltage reg and LED removed)
  - generic 3-pin 433,92 MHz transmitter
  - BH1750 i2c Luxmeter
  - VEML6070 i2c Ultra-Violet radiation sensor
  - HTU21DF i2c Temperature and Humidity sensor (3.3V no i2c pullup)

Channel allocations (can be customized):

  - 0 = UV Index
  - 2 = Temperature and Humidity %
  - 3 = Lux (sent scaled as Temperature) and Battery (sent scaled as Humidity %)
  
TARGET: ATmega 168 / 8 MHz @ 3.0V = 2.81 mA idle / 6.42 mA tx peak

WARNING: with _DEBUG_ enabled, sketch uses up to 80% of AT168P Dynamic Memory. 
         This is marginal for stability and resources availability. 
         Comment _DEBUG_ for release version.
         Upgrade to 328P should be also possible.

Tested succesfully with WMR88 and WMR200 OS base stations.

Last updated on November 28, 2021
