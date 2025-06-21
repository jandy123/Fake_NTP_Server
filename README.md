# Fake_NTP_Server

Arduino fake NTP server for [OpenCamera-MultiSync](https://github.com/jandy123/OpenCamera-MultiSync) running on an Esp8266. Fake_NTP_Server sets up a Wi-Fi access point and serves to connected clients a fake time based on a fixed reference time and the time elapsed since the device booted. The device is meant to be battery-powered and portable. It has been implemented using a LOLIN/Wemos D1 mini and a cheap TP4056 LiPo battery charger modules.

The following projects are used:

1. [ESP32 NTP Server](https://github.com/DennisSc/PPS-ntp-server)
2. [Arduino Time Library](https://github.com/DennisSc/microTime)

#### Setup

1. Change as desired the APSSID and pass in Fake_NTP_Server.ino.
2. Compile and upload on the device using Arduino.
3. For usage, follow instructions on [OpenCamera-MultiSync](https://github.com/jandy123/OpenCamera-MultiSync).





