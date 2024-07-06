#include <stdint.h>
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
//#include <EEPROM.h>
#include <avr/boot.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "RTClib.h"
#include <Ethernet.h>
#include <EthernetUdp.h>

SoftwareSerial GPSSerial(5, 6); // RX, TX
TinyGPSPlus gps;
RTC_DS3231 rtc;

//#define DEBUG_EN        //comment out if you don't want any Soft serial output
uint8_t mac[6] = {MAC};
#define SYSCLOCK 16000000
#define WDT_Time        WDTO_2S
#define SERIAL_BAUD     115200
#define SERIAL_BUF_MAX  60
#define SERIALFLUSH()   Serial.flush()
#define WDT_Time WDTO_2S

#ifdef DEBUG_EN
#define DEBUG(input)    Serial.print(input)
#define DEBUGln(input)  Serial.println(input)
#else
#define DEBUG(input)
#define DEBUGln(input)
#endif

#define SSEth           8
#define REth            7

#define fgpsOk 0
#define fStart 1
#define fOutDate 7
#define NTP_PORT 123 // стандартный порт, не менять

//#define Reset() asm("JMP 0")
void (* Reset) (void) = 0;
void setup_watchdog(int timerPrescaler);
void getDateTimeGPS();
void processNTP();
void displayInfo();
void serialPars();

// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
// this is NTP time (seconds since Jan 1 1900):  Unixtime + seventyYears!!!!
const unsigned long seventyYears = 2208988800UL;
static const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
String gpsData; // NMEA packet from GPS
long last = 0;
long lastRTCSet = 0;
int interval = 1000;
static uint8_t flags = 0;
uint32_t timestamp, tempval;