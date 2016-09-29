
#include <SPI.h>
#include <SD.h>

//#include <TinyGPS>


//SoftwareSerial geigerSerial(2, 3); //rx, tx
String burst;
float gpsDataArray[] = {0,0,0,0,0,0}; // date, time, lat, long, speed, alt

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
float Date;
float Lat;
float Long;
float Time;
float Speed;
float Altitude;
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
  strcpy(filename, "sofLog00.TXT");
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
    Serial.print(F("LOCATION   Fix Age="));
    //Serial.print(gps.location.age());
    //Serial.print(F("ms Raw Lat="));
    //Serial.print(gps.location.rawLat().negative ? "-" : "+");
    //Serial.print(gps.location.rawLat().deg);
    //Serial.print("[+");
    //Serial.print(gps.location.rawLat().billionths);
    //Serial.print(F(" billionths],  Raw Long="));
    //Serial.print(gps.location.rawLng().negative ? "-" : "+");
    //Serial.print(gps.location.rawLng().deg);
    //Serial.print("[+");
    //Serial.print(gps.location.rawLng().billionths);
    
    
    Lat = gps.location.lat();
    gpsDataArray[2] = Lat;
    
    Long = gps.location.lng();
    gpsDataArray[3] = Long;
    
    
    Serial.print(F(" billionths],  Lat="));
    Serial.print(Lat);
    Serial.print(F(" billionths],  Long="));
    Serial.print(Long);
    Serial.println(gps.location.lng(), 6);
  }

  else if (gps.date.isUpdated())
  {
    
    Date = gps.date.value();
    gpsDataArray[0] = Date;
    Serial.print("RAW DATE ");
    Serial.println(Time);
    
//    Serial.print(F("DATE       Fix Age="));
    
//    Serial.print(gps.date.age());
//    Serial.print(F("ms Raw="));
//    Serial.println(gps.date.value());
//    Serial.print(F(" Year="));
//    Serial.print(gps.date.year());
//    Serial.print(F(" Month="));
//    Serial.print(gps.date.month());
//    Serial.print(F(" Day="));
//    Serial.println(gps.date.day());
  }

  else if (gps.time.isUpdated())
  {
    Serial.print(F("TIME       Fix Age="));
    Serial.print(gps.time.age());

    Time = gps.time.value();
    gpsDataArray[1] = Time;


    Serial.print(F("ms Raw="));
    Serial.print(gps.time.value());
    
//    Serial.print(F(" Hour="));
//    Serial.print(gps.time.hour());
//    Serial.print(F(" Minute="));
//    Serial.print(gps.time.minute());
//    Serial.print(F(" Second="));
//    Serial.print(gps.time.second());
//    Serial.print(F(" Hundredths="));
//    Serial.println(gps.time.centisecond());
  }

  else if (gps.speed.isUpdated())
  {
    Serial.print(F("SPEED      Fix Age="));
//    Serial.print(gps.speed.age());
//    Serial.print(F("ms Raw="));
//    Serial.print(gps.speed.value());
//    Serial.print(F(" Knots="));
//    Serial.print(gps.speed.knots());
//    Serial.print(F(" MPH="));
//    Serial.print(gps.speed.mph());

    Speed = gps.speed.mps();
    gpsDataArray[4] = Speed;
    
    Serial.print(F(" m/s="));
    Serial.println(Speed);
//    Serial.println(gps.speed.mps());
//    Serial.print(F(" km/h="));
//    Serial.println(gps.speed.kmph());
  }



/*
  else if (gps.course.isUpdated())
  {
    Serial.print(F("COURSE     Fix Age="));
    Serial.print(gps.course.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.course.value());
    Serial.print(F(" Deg="));
    Serial.println(gps.course.deg());
  }
*/  

  else if (gps.altitude.isUpdated())
  {
    //Serial.print(F("ALTITUDE   Fix Age="));
    //Serial.print(gps.altitude.age());
    //Serial.print(F("ms Raw="));
    //Serial.print(gps.altitude.value());


    Altitude = gps.altitude.meters();
    gpsDataArray[5] = Altitude;
    
    Serial.print("Meters=");
    Serial.println(Altitude);

    
    //Serial.println(gps.altitude.meters());
    //Serial.print(F(" Miles="));
    //Serial.print(gps.altitude.miles());
    //Serial.print(F(" KM="));
    //Serial.print(gps.altitude.kilometers());
    //Serial.print(F(" Feet="));
    //Serial.println(gps.altitude.feet());
  }

//  else if (gps.satellites.isUpdated())
//  {
//    Serial.print(F("SATELLITES Fix Age="));
//    Serial.print(gps.satellites.age());
//    Serial.print(F("ms Value="));
//    Serial.println(gps.satellites.value());
//  }

//  else if (gps.hdop.isUpdated())
//  {
//    Serial.print(F("HDOP       Fix Age="));
//    Serial.print(gps.hdop.age());
//    Serial.print(F("ms Value="));
//    Serial.println(gps.hdop.value());
//  }


  else if (millis() - last > 5000)
  {
    Serial.println();
/*    if (gps.location.isValid())
    {
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      double distanceToLondon =
        TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
      double courseToLondon =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
      Serial.print(F("LONDON     Distance="));
      Serial.print(distanceToLondon/1000, 6);
      Serial.print(F(" km Course-to="));
      Serial.print(courseToLondon, 6);
      Serial.print(F(" degrees ["));
      Serial.print(TinyGPSPlus::cardinal(courseToLondon));
      Serial.println(F("]"));
    }
*/
//    Serial.print(F("DIAGS      Chars="));
//    Serial.print(gps.charsProcessed());
//    Serial.print(F(" Sentences-with-Fix="));
//    Serial.print(gps.sentencesWithFix());
//    Serial.print(F(" Failed-checksum="));
//    Serial.print(gps.failedChecksum());
//    Serial.print(F(" Passed-checksum="));
//    Serial.println(gps.passedChecksum());

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

    last = millis();
    Serial.println();

  }

  //writeSD(gpsDataArray);

  digitalWrite(8, HIGH);
  logfile.print(gpsDataArray[0]);
  logfile.print(", ");
  logfile.print(gpsDataArray[1]);
  logfile.print(", ");
  logfile.print(gpsDataArray[2]);
  logfile.print(", ");
  logfile.print(gpsDataArray[3]);
  logfile.print(", ");
  logfile.println(gpsDataArray[4]);
  logfile.print(", ");
  logfile.println(gpsDataArray[5]);
  digitalWrite(8, LOW);

  logfile.flush();
}

void writeSD(float gpsvals[]) {
  // in this loop we will write to the SD card
  digitalWrite(8, HIGH);
  logfile.print(gpsvals[0]);
  logfile.print(", ");
  logfile.print(gpsvals[1]);
  logfile.print(", ");
  logfile.print(gpsvals[2]);
  logfile.print(", ");
  logfile.print(gpsvals[3]);
  logfile.print(", ");
  logfile.println(gpsvals[4]);
  logfile.print(", ");
  logfile.println(gpsvals[5]);
  digitalWrite(8, LOW);

  logfile.flush();

  exit;
}

void readGPS() {
  // in this loop we will read the GPS and send it to the log
  // it is probably best if we keep it persistant in a variable
  // based on TinyGPS++

}

void readIMU() {
  // read the Arduino 101 IMU

}

void pollGeigerCounter() {
  // poll the Geiger Counter for changes >> Use as interrupt to trigger a event read?

}


