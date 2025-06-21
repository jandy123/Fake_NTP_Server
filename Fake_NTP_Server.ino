#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "microTime.h"
#include "microTimeLib.h"
#include "DateTime.h"

//#define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTln(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) do {} while (0)
#define DEBUG_PRINTln(x) do {} while (0)
#endif

//check battery level every 5 minutes
//see https://github.com/G6EJD/LiPo_Battery_Capacity_Estimator/blob/master/ReadBatteryCapacity_LIPO.ino
#define PERIOD 5*60*1000UL

const String APSSID = "Esp8266_NTPS";
const String pass = "123456";

//UDP server (ETH.h uses WifiUDP class)
WiFiUDP Udp;

// global NTP timestamp storage variables
DateTime referenceTime;
DateTime receiveTime;
DateTime transmitTime;
byte origTimeTs[9];

unsigned long prev_millis; 

// offset used to adjust PPS clock pulse to match known good time sources
#define TIME_OFFSET_USEC 503900 

//interval in seconds to sync system time with gps
#define TIMESYNC_INTERVAL 15

static bool SerialMirror = 0;

// built-in LED
#define LED_PIN LED_BUILTIN

// NTP port and packet buffer
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];

/*
// fake time provider
time_t faketimeprovider(){
  
    tmElements_t timeinfo;
    timeinfo.Hour = 22;
    timeinfo.Minute = 06;
    timeinfo.Second = 45;
    timeinfo.Month = 9;
    timeinfo.Year = 23;
    time_t time = makeTime(timeinfo);
    //Serial.println(time);
    return time;
}
*/

void printSysTime()
{
    long diff2 = microsecond();

    DEBUG_PRINT("sysTime (UTC): ");
    if (hour() < 10)
        DEBUG_PRINT("0");
    DEBUG_PRINT(hour());
    DEBUG_PRINT(":");
    if (minute() < 10)
        DEBUG_PRINT("0");
    DEBUG_PRINT(minute());
    DEBUG_PRINT(":");
    if (second() < 10)
        DEBUG_PRINT("0");
    DEBUG_PRINT(second());

    // 1*10E-6... this is 1 microsecond precision
    DEBUG_PRINT(",");
    if (diff2 < 10)
        DEBUG_PRINT("00000");
    else if (diff2 < 100)
        DEBUG_PRINT("0000");
    else if (diff2 < 1000)
        DEBUG_PRINT("000");
    else if (diff2 < 10000)
        DEBUG_PRINT("00");
    else if (diff2 < 100000)
        DEBUG_PRINT("0");
    
    
    DEBUG_PRINT(diff2);
    DEBUG_PRINTln();
}

void updateDateTime(DateTime &dt)
{
    uint32_t _microsfraction;
    time_t _now = now(_microsfraction);
    _microsfraction += TIME_OFFSET_USEC;
    while (_microsfraction >= 1000000)
    {
        _microsfraction -= 1000000;
        _now++;
    }
    
    DateTime _dt(_now, _microsfraction);
    dt = _dt;
}

void configureSoftAP() 
{
  DEBUG_PRINTln("Configuring AP: " + String(APSSID));
  
  /* This seems to stop crashing the ESP32 if in SoftAP mode */
  WiFi.enableAP(true);
  delay(100);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(APSSID.c_str(), pass.c_str(), 1, 0, 4); //max  4 connections, channel 1
  //esp_wifi_set_ps(WIFI_PS_NONE); //LUK: no difference???
  delay(500); // Without delay the IP address is sometimes blank

  IPAddress ip_addr = WiFi.softAPIP();
  DEBUG_PRINT("AP IP address: ");
  DEBUG_PRINTln(ip_addr);
}

float checkBattery() 
{
    float voltage = analogRead(A0) / 1023.0f * 4.09f;
    digitalWrite(LED_PIN, (voltage <= 3.7f) ? LOW : HIGH);

    DEBUG_PRINT("battery voltage: ");
    DEBUG_PRINTln(voltage);

    return voltage;
}

void setup() 
{
    //btStop(); // turn off ble radio

#ifdef DEBUG
    Serial.begin(115200);
#endif

    DEBUG_PRINTln("\r\n\r\n\r\nNTP Server started.");
    
    configureSoftAP();

    //set fake time for debugging purposes
    //const unsigned long DEFAULT_TIME = 1357041600UL;
    //setTime(DEFAULT_TIME);
    setTime(22, 0, 0, 23, 9, 23); //LUK: time:22:00:00 date:23-09-20203


    DEBUG_PRINTln("getting NTP reference time:\r\n");
    updateDateTime(referenceTime);
    referenceTime.print();

    Udp.begin(NTP_PORT); // start udp server


    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_PIN, OUTPUT);
    checkBattery();

    prev_millis = millis();
}


uint64_t DateTimeToNtp64(DateTime dt) 
{
    uint64_t ntpts;

    ntpts = (((uint64_t)dt.ntptime()) << 32);
    ntpts |= (uint64_t)(dt.microsfraction() * 4294.967296);

    return (ntpts);
}

// send NTP reply to the given address
void sendNTPpacket(IPAddress remoteIP, int remotePort) 
{
  // set all bytes in the packet buffer to 0
  //memset(packetBuffer, 0, NTP_PACKET_SIZE);

  // Initialize values needed to form NTP request
  // (see URL in readme.md for details on packet fields)
  
  // LI: 0, Version: 4, Mode: 4 (server)
  //packetBuffer[0] = 0b00100100;
  // LI: 0, Version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;
  
  // Polling Interval
  packetBuffer[2] = 4;

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s 
  packetBuffer[3] = 0xF7;
  
  // 8 bytes for Root Delay & Root Dispersion
  // root delay
  packetBuffer[4] = 0; 
  packetBuffer[5] = 0;
  packetBuffer[6] = 0; 
  packetBuffer[7] = 0;
  
  // root dispersion
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0x50;
  
  
  //time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

  
  // Reference Time
  uint64_t refT = DateTimeToNtp64(referenceTime);
  
  packetBuffer[16] = (int)((refT >> 56) & 0xFF);
  packetBuffer[17] = (int)((refT >> 48) & 0xFF);
  packetBuffer[18] = (int)((refT >> 40) & 0xFF);
  packetBuffer[19] = (int)((refT >> 32) & 0xFF);
  packetBuffer[20] = (int)((refT >> 24) & 0xFF);
  packetBuffer[21] = (int)((refT >> 16) & 0xFF);
  packetBuffer[22] = (int)((refT >> 8) & 0xFF);
  packetBuffer[23] = (int)(refT & 0xFF);
 
  // Origin Time
  //copy old transmit time to origtime 

  for (int i = 24; i < 32; i++)
  {
        packetBuffer[i] = origTimeTs[i-24];
        //Serial.write(origTimeTs[i-24]);
  }

  // write Receive Time to bytes 32-39
  refT = DateTimeToNtp64(receiveTime);
  
  packetBuffer[32] = (int)((refT >> 56) & 0xFF);
  packetBuffer[33] = (int)((refT >> 48) & 0xFF);
  packetBuffer[34] = (int)((refT >> 40) & 0xFF);
  packetBuffer[35] = (int)((refT >> 32) & 0xFF);
  packetBuffer[36] = (int)((refT >> 24) & 0xFF);
  packetBuffer[37] = (int)((refT >> 16) & 0xFF);
  packetBuffer[38] = (int)((refT >> 8) & 0xFF);
  packetBuffer[39] = (int)(refT & 0xFF);
  
  
  // get current time + write  as Transmit Time to bytes 40-47
  updateDateTime(transmitTime);
  refT = DateTimeToNtp64(transmitTime);
  
  packetBuffer[40] = (int)((refT >> 56) & 0xFF);
  packetBuffer[41] = (int)((refT >> 48) & 0xFF);
  packetBuffer[42] = (int)((refT >> 40) & 0xFF);
  packetBuffer[43] = (int)((refT >> 32) & 0xFF);
  packetBuffer[44] = (int)((refT >> 24) & 0xFF);
  packetBuffer[45] = (int)((refT >> 16) & 0xFF);
  packetBuffer[46] = (int)((refT >> 8) & 0xFF);
  packetBuffer[47] = (int)(refT & 0xFF);
  
  // send reply:
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();

}

//LUK: debug with sudo tcpdump -i wlp0s20f3 -vv -q -X -n udp port 123 on laptop
void sendBatteryVoltage(IPAddress remoteIP, int remotePort, float val)
{
    int v = (int)(val * 1000.0f);

    packetBuffer[0] = (int)((v >> 24) & 0xFF);
    packetBuffer[1] = (int)((v >> 16) & 0xFF);
    packetBuffer[2] = (int)((v >> 8) & 0xFF);
    packetBuffer[3] = (int)(v & 0xFF);
/*
    packetBuffer[0] =  0x44; //D
    packetBuffer[1] =  0x45; //E
    packetBuffer[2] =  0x41; //A
    packetBuffer[3] =  0x44; //D
 */   
    // send reply:
    Udp.beginPacket(remoteIP, remotePort);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}

void loop() 
{
    if(millis() - prev_millis >= PERIOD ) {
        prev_millis += PERIOD;

        float v = checkBattery();

        //sendBatteryVoltage(remoteIP(192, 168, 4, 100), NTP_PORT, v);       
        //return;
    }


    if (SerialMirror)
    {
        if (Serial1.available())
        {
            Serial.write(Serial1.read());
        }

        if (Serial.available())
        {
            Serial1.write(Serial.read());
        }

    }
    else
    {

        // process NTP requests
        IPAddress remoteIP;
        int remotePort;

        int packetSize = Udp.parsePacket();

        if (packetSize) // we've got a packet 
        {
            updateDateTime(receiveTime);

            
            //store sender ip and port for later use
            remoteIP = Udp.remoteIP();
            remotePort = Udp.remotePort();

           /* 
            DEBUG_PRINT("Received UDP packet with ");
            DEBUG_PRINT(packetSize);
            DEBUG_PRINT(" bytes size - ");
            DEBUG_PRINT("SourceIP ");
            
            for (uint8_t i =0; i < 4; i++)
            {
                DEBUG_PRINT(remoteIP[i], DEC);
                if (i < 3)
                {
                    DEBUG_PRINT(".");
                }
            }
            
            DEBUG_PRINT(", Port ");
            DEBUG_PRINTln(remotePort);
            */
            
            DEBUG_PRINT("query: ");
            DEBUG_PRINT(receiveTime.toString());
            DEBUG_PRINT(",");
            DEBUG_PRINT(receiveTime.microsfraction());
            DEBUG_PRINTln();

            //digitalWrite(LED_PIN, HIGH);

            
            // We've received a packet, read the data from it
            // read the packet into the buffer
            Udp.read(packetBuffer, NTP_PACKET_SIZE);

            //get client transmit time (becomes originTime in reply packet)
            for (int i = 0; i < 8; i++)
            {
                origTimeTs[i] = packetBuffer[40+i];
            }
            
            //send NTP reply
            sendNTPpacket(remoteIP, remotePort);
            DEBUG_PRINT("reply: ");
            DEBUG_PRINT(transmitTime.toString());
            DEBUG_PRINT(",");
            DEBUG_PRINTln(transmitTime.microsfraction());
            //digitalWrite(LED_PIN, LOW);
            
            //output "done"
            //updateLCDtime();
            DEBUG_PRINTln("NTP reply sent.\r\n*********************************");
            
        }
    }
    
    // put your main code here, to run repeatedly:

}
