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
}

void loop() {
  //long now = (long)millis();
  wdt_reset();
  /*if ((now - interval) > 1000) {
    Serial.print("Tttt...\n");
    interval = (long)millis();
  }*/
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) displayInfo();
  }
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
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
  } else {
    Serial.print(F("INVALID"));
  }
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
  Serial.println();
}