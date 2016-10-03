
#include <SPI.h>
#include <SD.h>


//initialize variables

//SoftwareSerial geigerSerial(2, 3); //rx, tx
float gpsDataArray[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // date, time, lat, long, speed, alt

int geigerCPS = 0;    //this is the variable where the Geiger pulses/ Counts Per Second will be counted and stored

//updated
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

byte geigerPin = 12;

//****Feather M0 specific code - add UART*******


//For communication with the GPS
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
 
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

//For communication with the Geiger Counter
//Uart Serial5 (&sercom5, 6, 7, SERCOM_RX_PAD_2, UART_TX_PAD_3);
// 
//void SERCOM5_Handler()
//{
//  Serial1.IrqHandler();
//}

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

//set up TinyGPS++ for the serial communication with the GPS unit
static const uint32_t GPSBaud = 9600;

//Set up Geiger SERCOM settings
static const int GeigerBaud = 9600;
String geigerCounterString;


// Set up the GPS data read to be stored as float values
float Date;
float Lat;
float Long;
float Time;
float Speed;
float Altitude;

// The TinyGPSLOG++ object
TinyGPSPlus gps;

// For stats that happen every 5 seconds
unsigned long last = 0UL;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(GPSBaud);
  Serial1.begin(GeigerBaud);
  pinMode(13, OUTPUT);
  
  // Assign pins SERCOM functionality
  pinPeripheral(6, PIO_SERCOM);
  pinPeripheral(7, PIO_SERCOM);
  
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

Serial.println(TinyGPSPlus::libraryVersion());


//****************SD CARD Datalog Setup*********************


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
  
  //********************Set up Datalog header for GPS************************


  logfile.print("Date");
  logfile.print(", ");
  logfile.print("Time");
  logfile.print(", ");
  logfile.print("Latitude");
  logfile.print(", ");
  logfile.print("Longitude");
  logfile.print(", ");
  logfile.print("Speed (m/s)");
  logfile.print(", ");
  logfile.print("Altitude");
  logfile.print(", ");
  logfile.println("Geiger CPS");



  //****************************Geiger Counter********************

  pinMode(12, INPUT);  //set up the pin for input from the Geiger counter pulse
  
}

void loop()
{

  //count Geiger pulses per second - will reset to zero when writing GPS data
//  if (digitalRead(12) == HIGH){
//      geigerCPS += 1;
//  }

  if(Serial1.available() > 0)
    {
//      if(Serial1.read() == '\n')
//      {
        geigerCounterString = Serial1.read();
 //       Serial.println(geigerCounterString);
        Serial.println(Serial1.parseInt());  
//        
        //Serial.println(Serial1.parseInt());  
        //Serial.println(Serial1.parseInt());  
        //Serial.println(Serial1.parseInt());  
        //Serial.println(Serial1.parseInt());  
        //Serial.println(Serial1.parseInt());  
//      }
    }

  // Read incoming characters from the GPS serial output
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());

  if (gps.location.isUpdated())
  {
    Lat = (float) gps.location.lat();
    gpsDataArray[2] = Lat;
    
    Long = (float) gps.location.lng();
    gpsDataArray[3] = Long;
  }

  else if (gps.date.isUpdated())
  {
    
    Date = gps.date.value();
    gpsDataArray[0] = Date;
  }

  else if (gps.time.isUpdated())
  {
    Time = gps.time.value();
    gpsDataArray[1] = Time;
  }

  else if (gps.speed.isUpdated())
  {
    Speed = gps.speed.mps();
    gpsDataArray[4] = Speed;
  } 

  else if (gps.altitude.isUpdated())
  {
    Altitude = gps.altitude.meters();
    gpsDataArray[5] = Altitude;
  }


  else if (millis() - last > 1000)
  {

    
    //Serial.println(Serial3.read());
    
    Serial.print("RAW DATE = ");
    Serial.println(Date);
  
    Serial.print("RAW TIME = ");
    Serial.println(Time);  

    Serial.print("Lat = ");
    Serial.println(Lat, 6);
    
    Serial.print("Long = ");
    Serial.println(Long, 6);

    Serial.print("Speed (m/s)= ");
    Serial.println(Speed);
    
    Serial.print("Meters= ");
    Serial.println(Altitude);

    
    Serial.print("CPS = ");
    Serial.println(geigerCPS);

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

      
    digitalWrite(8, HIGH);
    logfile.print(gpsDataArray[0]);
    logfile.print(", ");
    logfile.print(gpsDataArray[1]);
    logfile.print(", ");
    logfile.print(gpsDataArray[2], 6);
    logfile.print(", ");
    logfile.print(gpsDataArray[3], 6);
    logfile.print(", ");
    logfile.print(gpsDataArray[4]);
    logfile.print(", ");
    logfile.print(gpsDataArray[5]);
    logfile.print(", ");
    logfile.println(geigerCPS);
    digitalWrite(8, LOW);
    logfile.flush();

    geigerCPS = 0;


    //writeSD(gpsDataArray);

  
    last = millis();
    Serial.println();

  }

}

void writeSD(float gpsvals[]) {

  //debug
//  digitalWrite(8, HIGH);
//  Serial.println("begin");
//  Serial.println(gpsvals[0]);
//  Serial.println(gpsvals[1]);
//  Serial.println(gpsvals[2]);
//  Serial.println(gpsvals[3]);
//  Serial.println(gpsvals[4]);
//  Serial.println(gpsvals[5]);
//  Serial.println("end");
//  digitalWrite(8, LOW);


  
  // in this loop we will write to the SD card
  digitalWrite(8, HIGH);
  logfile.print(gpsvals[0]);
  logfile.print(", ");
  logfile.print(gpsvals[1]);
  logfile.print(", ");
  logfile.print(gpsvals[2], 6);
  logfile.print(", ");
  logfile.print(gpsvals[3], 6);
  logfile.print(", ");
  logfile.print(gpsvals[4]);
  logfile.print(", ");
  logfile.println(gpsvals[5]);
  digitalWrite(8, LOW);

  logfile.flush();

  return;
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

  digitalRead(geigerPin);

}


