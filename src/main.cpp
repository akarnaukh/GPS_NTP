#include <main.h>

void setup() {
  wdt_disable();
  Serial.begin(SERIAL_BAUD);
  delay(200);
  SERIALFLUSH();
  GPSSerial.begin(9600);
  delay(200);
  GPSSerial.flush();
  // LM: These sentences are the same for the NEO-M8N   -  Comments added
  GPSSerial.write("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");   // Lat/Lon data
  GPSSerial.write("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");   // Vector track and speed over ground
  GPSSerial.write("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");   // Detailed satellite data
  GPSSerial.write("$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n");   // Fix information 
  GPSSerial.write("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");   // Overall satellite data
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //rtc.adjust(DateTime(2000, 0, 0, 0, 0, 0));
   }
   bitSet(flags,fStart);
   pinMode(13,OUTPUT);
   digitalWrite(13,LOW);
}

void loop() {
  long now = (long)millis();
  wdt_reset();
  serialPars();
  getDateTimeGPS();
  if ((now - last) > interval) {
    if (bitRead(flags,fOutDate)) displayInfo();
    digitalWrite(13,LOW);
    last = (long)millis();
  }
  if (bitRead(flags,fgpsOk) && bitRead(flags,fStart)) {
    Serial.println("First set RTC from GPS...");
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(YYYY, MM, DD, hh, mm, ss));
    rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
    bitClear(flags,fStart);
  }
  if (bitRead(flags,fgpsOk) && ((now - lastRTCSet) > 120000) ) {
    rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
    Serial.println("RTC set from GPS...");
    lastRTCSet = (long)millis();
  }
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void getDateTimeGPS() {
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      bitSet(flags,fgpsOk);
      digitalWrite(13,HIGH);
    }
  }
}

void displayInfo() {
  //Serial.print(F("Sats=")); Serial.print(gps.satellites.value());
  //Serial.print(F(" Alt=")); Serial.print(gps.altitude.isValid());
  Serial.print(F("Location: ")); 
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid() && (gps.date.year() > 2000)) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  
    Serial.print(F(" "));
    if (gps.time.isValid()) {
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(gps.time.second());
      Serial.print(F("."));
      if (gps.time.centisecond() < 10) Serial.print(F("0"));
      Serial.print(gps.time.centisecond());
    } else {
      Serial.print(F("INVALID"));
    }
  } else {
    Serial.print(F("INVALID"));
  }
  Serial.println();
}

void serialPars() {
    if (Serial.available() > 0) {
        char input = Serial.read();
        if (input == 'd') {
          if (bitRead(flags,fOutDate)) {
            bitClear(flags, fOutDate);
          } else {
            bitSet(flags, fOutDate);
          }
          Serial.print("Out Date - ");
          if (bitRead(flags, fOutDate)) Serial.println("On"); else Serial.println("Off");
        }
        if (input == 'r') {
          DateTime nowRtc = rtc.now();
          char buf2[] = "YYYY-MM-DD-hh:mm:ss";
          Serial.println(nowRtc.toString(buf2));
        }
        if (input == 'u') {
          DateTime nowRtc = rtc.now();
          Serial.print("Unixtime - ");
          Serial.print(nowRtc.unixtime());
          Serial.print("s");
        }
    }
}