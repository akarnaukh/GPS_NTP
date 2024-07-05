#include <stdint.h>
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
//#include <EEPROM.h>
#include <avr/boot.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

SoftwareSerial GPSSerial(10, 11); // RX, TX
TinyGPSPlus gps;
void displayInfo();

//#define DEBUG_EN        //comment out if you don't want any Soft serial output
uint8_t mac[6] = {MAC};
#define SYSCLOCK 16000000
#define WDT_Time        WDTO_2S
#define SERIAL_BAUD     115200
#define SERIAL_BUF_MAX  60
#define SERIALFLUSH()   Serial.flush()

#ifdef DEBUG_EN
#define DEBUG(input)    Serial.print(input)
#define DEBUGln(input)  Serial.println(input)
#else
#define DEBUG(input)
#define DEBUGln(input)
#endif

//#define Reset() asm("JMP 0")
void (* Reset) (void) = 0;

int interval = 1000;