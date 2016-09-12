//updated
#include <TinyGPS++.h>
//#include <SoftwareSerial.h>         //not available on ARM Feather M0
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function



//****Feather M0 specific code - add UART*******

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
 
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

//**********************************************

//************SD CARD************
// Set the pins used
#define cardSelect 4

File logfile;

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

//*************************

/*
   This sample code demonstrates just about every built-in operation of TinyGPSLOG++ (TinyGPSLOGPlus).
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPSLOG device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 11, TXPin = 10;
//9600 for 20U7
static const uint32_t GPSBaud = 9600;
String Location;
String Lat;
String Long;
String Time;
String Speed;
String Altitude;
String GPSLOG;
String Comma;

// The TinyGPSLOG++ object
TinyGPSPlus gps;

// The serial connection to the GPSLOG device
//SoftwareSerial ss(RXPin, TXPin);

// For stats that happen every 5 seconds
unsigned long last = 0UL;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(GPSBaud);
pinMode(13, OUTPUT);
  
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

Serial.println(TinyGPSPlus::libraryVersion());
  Comma = ", ";


//****************SD CARD*********************


  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

   logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }

  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  
//********************************************
  
}

void loop()
{
 
  // Dispatch incoming characters
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());


  if (gps.location.isUpdated())
  {
   
   Lat = (gps.location.lat());
   GPSLOG = Lat;
   GPSLOG.concat(Comma);
   Long = (gps.location.lng());
   GPSLOG.concat(Long);
   GPSLOG.concat(Comma);
  }


  else if (gps.time.isUpdated())
  {
    Time = (gps.time.value());
    GPSLOG.concat(Time);
    GPSLOG.concat(Comma);
  }

  else if (gps.speed.isUpdated())
  {
    Speed = (gps.speed.kmph());
    GPSLOG.concat(Speed);
    GPSLOG.concat(Comma);
  }

  else if (gps.altitude.isUpdated())
  {
    Altitude = (gps.altitude.kilometers());
    GPSLOG.concat(Altitude);
   //Serial.println(GPSLOG);
  }


  else if (millis() - last > 5000)
  {
    Serial.println();
    Serial.print(F("DIAGS      Chars="));
    Serial.print(gps.charsProcessed());
    Serial.print(F(" Sentences-with-Fix="));
    Serial.print(gps.sentencesWithFix());
    Serial.print(F(" Failed-checksum="));
    Serial.print(gps.failedChecksum());
    Serial.print(F(" Passed-checksum="));
    Serial.println(gps.passedChecksum());

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPSLOG data.  Check wiring."));

    last = millis();
    Serial.println();
 
  
  }

//****************SD****************************
  digitalWrite(8, HIGH);
  logfile.println(GPSLOG);
  Serial.println(GPSLOG);
  digitalWrite(8, LOW);

  logfile.flush();
  delay(500);
  //**********************************************


}
