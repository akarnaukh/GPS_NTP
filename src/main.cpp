#include <main.h>

EthernetUDP Udp;

void setup() {
  wdt_disable();
  pinMode(REth, OUTPUT);
  digitalWrite(REth, LOW);
  Serial.begin(SERIAL_BAUD);
  delay(200);
  SERIALFLUSH();
  GPSSerial.begin(9600);
  delay(200);
  digitalWrite(REth, HIGH);
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
  Ethernet.init(SSEth);
  Ethernet.begin(mac);
  bitSet(flags,fStart);
  //pinMode(13,OUTPUT);
  //digitalWrite(13,LOW);
  setup_watchdog(WDT_Time);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet not found. :(");
  } else {
    Serial.print("Ehernet found - ");
    Serial.println(Ethernet.hardwareStatus());
  }
  Serial.print("Link status - ");
  Serial.println(Ethernet.linkStatus());
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  Udp.begin(NTP_PORT);
  wdt_enable(WDT_Time);
  sei();
}

void setup_watchdog(int timerPrescaler) {
    if (timerPrescaler > 9)
        timerPrescaler = 9;  // Correct incoming amount if need be
    byte bb = timerPrescaler & 7;
    if (timerPrescaler > 7)
        bb |= (1 << 5);
    // This order of commands is important and cannot be combined
    MCUSR &= ~(1 << WDRF);               // Clear the watch dog reset
    WDTCSR |= (1 << WDCE) | (1 << WDE);  // Set WD_change enable, set WD enable
    WDTCSR = bb;                         // Set new watchdog timeout value
    WDTCSR |= _BV(WDIE);
    // Set the interrupt enable, this will keep unit from resetting after each int
}

void loop() {
  long now = (long)millis();
  wdt_reset();
  serialPars();
  processNTP();
  getDateTimeGPS();
  if ((now - last) > interval) {
    if (bitRead(flags,fOutDate)) displayInfo();
    //digitalWrite(pinLed,LOW);
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
    char gpsread = GPSSerial.read();
    //if (bitRead(flags,fOutDate)) Serial.write(gpsread);
    if (gps.encode(gpsread)) {
      if (gps.location.isValid() && gps.date.isValid() && (gps.date.year() > 2000)) bitSet(flags,fgpsOk);
      //digitalWrite(pinLed,HIGH);
    }
  }
}

void processNTP() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress remote = Udp.remoteIP();
    int portNum = Udp.remotePort();

//DEBUG
    Serial.print("Received UDP packet size ");
    Serial.println(packetSize);
    Serial.print("From ");

    for (int i=0; i<4; i++) {
      Serial.print(remote[i], DEC);
      if (i<3) { Serial.print("."); }
    }
    Serial.print(", port ");
    Serial.println(portNum);
/*
    byte LIVNMODE = packetBuffer[0];
    Serial.print("  LI, Vers, Mode :");
    Serial.print(packetBuffer[0], HEX);

    byte STRATUM = packetBuffer[1];
    Serial.print("  Stratum :");
    Serial.print(packetBuffer[1], HEX);

    byte POLLING = packetBuffer[2];
    Serial.print("  Polling :");
    Serial.print(packetBuffer[2], HEX);

    byte PRECISION = packetBuffer[3];
    Serial.print("  Precision :");
    Serial.println(packetBuffer[3], HEX);

    for (int z=0; z<NTP_PACKET_SIZE; z++) {
      Serial.print(packetBuffer[z], HEX);
      if (((z+1) % 4) == 0) {
        Serial.println();
      }
    }
    Serial.println();
// END DEBUG
*/

    // Упаковываем данные в ответный пакет:
    packetBuffer[0] = 0b00100100;   // версия, режим
    packetBuffer[1] = 1;   // стратум
    packetBuffer[2] = 6;   // интервал опроса
    packetBuffer[3] = 0xFA; // точность

    packetBuffer[7] = 0; // задержка
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0; // дисперсия
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;
      
    DateTime nowRtc = rtc.now();
    //Serial.print("Unixtime - ");
    //Serial.println(nowRtc.unixtime());

    timestamp = nowRtc.unixtime() + seventyYears;
    //Serial.println((String)timestamp);
    tempval = timestamp;
    packetBuffer[12] = 71; //"G";
    packetBuffer[13] = 80; //"P";
    packetBuffer[14] = 83; //"S";
    packetBuffer[15] = 0; //"0";

    // Относительное время 
    packetBuffer[16] = (tempval >> 24) & 0xFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval) & 0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    // Копируем метку времени клиента 
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // Метка времени 
    packetBuffer[32] = (tempval >> 24) & 0xFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval) & 0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // Записываем метку времени 
    packetBuffer[40] = (tempval >> 24) & 0xFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;

    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;

    // Отправляем NTP ответ 
    Udp.beginPacket(remote, portNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
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
          Serial.println("s");
        }
        if (input == '?') {
          DateTime nowRtc = rtc.now();
          Serial.print("Link status - ");
          Serial.println(Ethernet.linkStatus());
          Serial.print("My IP address: ");
          Serial.println(Ethernet.localIP());
          Serial.print("GPS sttaus - ");
          if (bitRead(flags,fgpsOk)) Serial.println("Ok"); else Serial.println("Faild!");
          Serial.print("Unixtime - ");
          Serial.println(nowRtc.unixtime());
          char buf2[] = "MM-DD-YYYY / hh:mm:ss";
          Serial.println(nowRtc.toString(buf2));
        }
    }
}