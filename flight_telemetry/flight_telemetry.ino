


/*** to use the SD Card datalogger shield with an Arduino Mega:
     change: #define MEGA_SOFT_SPI 0
         to: #define MEGA_SOFT_SPI 1
     in the file libraries\S
     
     
     D\utility\Sd2Card.h 
***/
 
/*** Configuration options ***/

/* Sensors */
#define ENABLE_BMP085
#define ENABLE_ADXL
#define ENABLE_SDLOG

/* SD Shield */
#ifdef ENABLE_SDLOG
  #include <SD.h>
  #include "RTClib.h"
  RTC_DS1307 RTC;
  const int sdCS = 10;
#endif

/* I2C */
#if defined(ENABLE_BMP085) || defined(ENABLE_ADXL)
  #include <Wire.h>
  #include <Math.h>
  
  
#endif

/* Barometer */
#ifdef ENABLE_BMP085
  #include <Adafruit_BMP085.h>
  Adafruit_BMP085 dps;
  float Temperature = 0, Pressure = 0, Altitude = 0, seaPressure = 0;
  uint32_t nextBmp085 = 0;
#endif /* ENABLE_BMP085 */

/* Accelerometer */
#ifdef ENABLE_ADXL
  #include <ADXL345.h>
  ADXL345 accel;
#endif

void setup() 
{
  Serial.begin(57600);
  Serial.println("SAPHE Telemetry Payload");
  
  /* Setup the SD Log */
  #ifdef ENABLE_SDLOG
    Serial.print("Initializing SD Card...");
    pinMode(10, OUTPUT);
    if (!SD.begin(sdCS)) {
      Serial.println("Card failed, or not present");
      return;
    }
    Serial.println("card initialized.");
    
    File logFile = SD.open("LOG.csv", FILE_WRITE);
    if (logFile)
    {
      String header = "ID, timestamp, X, Y, Z, TempC, PressurePA, AltitudeM";
      logFile.println(header);
      logFile.close();
      Serial.println(header);
    }
    else
    {
      Serial.println("Couldn't open log file");
    }
  #endif /* ENABLE_SDLOG */
    
  /* I2C */
  #if defined(ENABLE_BMP085) || defined(ENABLE_ADXL)
    Wire.begin();
    delay(1);
  #endif
  
  #if defined(ENABLE_BMP085)
    dps.begin(BMP085_STANDARD); // Initialize for relative altitude    
    ;
    nextBmp085 = millis() + 1000;
    delay(500);
  #endif /* ENABLE_BMP085 */
  
  #if defined(ENABLE_ADXL)
    accel = ADXL345();
    if(accel.EnsureConnected()){
      Serial.println("Connected to ADXL345");
    }
    else {
      Serial.println("Could not connect to ADXL");
    }   
    accel.EnableMeasurements();
    
    #endif
  
  delay(1000);
};

void loop()
{
  #ifdef ENABLE_BMP085
    if (millis() > nextBmp085) {
      Temperature = dps.readTemperature();
      Pressure = dps.readPressure();
      Altitude = dps.readAltitude();
      seaPressure = dps.readSeaPressure(10); //change this 10 to be the altitude in meters as reported by gps
      
      Serial.print("  Temp(C):");
      Serial.print(Temperature);
      Serial.print("  Pressure(Pa):");
      Serial.print(Pressure);
      Serial.print("  Alt(m):");
      Serial.print(Altitude);
      Serial.print("  Calc Sealevel Pressure:");
      Serial.println(seaPressure);

    }
  #endif 
  
  #if defined(ENABLE_ADXL)
    AccelerometerRaw raw = accel.ReadRawAxis();
    int xAxisRawData = raw.XAxis;
    int yAxisRawData = raw.YAxis;
    int zAxisRawData = raw.ZAxis;
    AccelerometerScaled scaled = accel.ReadScaledAxis();
    float xAxisGs = scaled.XAxis;
    
    Serial.print("X: ");
    Serial.print(raw.XAxis);
    Serial.print("   Y: ");
    Serial.print(raw.YAxis);
    Serial.print("   Z: ");
    Serial.println(raw.ZAxis);
  #endif
  
  #ifdef ENABLE_SDLOG
    DateTime now = RTC.now();
    String record;
    String comma = ",";
    String colon = ":";
    String slash = "/";
    String space = " ";
    String utime;
    char temp[6];
    dtostrf(Temperature, 4, 2, temp); 
    char press[9];
    dtostrf(Pressure, 8, 1, press); 
    char alt[8];
    dtostrf(Altitude, 4, 1, alt); 
    char spress[9];
    dtostrf(seaPressure, 8, 1, spress);
    utime = (String) now.unixtime();
    char ryear[5];
    dtostrf(now.year(), 4, 0, ryear);
    char rmonth[3];
    dtostrf(now.month(), 1, 0, rmonth);
    char rday[3];
    dtostrf(now.day(), 1, 0, rday);
    char rhour[3];
    dtostrf(now.hour(), 2, 0, rhour);
    char rmin[3];
    dtostrf(now.minute(), 2, 0, rmin);
    char rsec[3];
    dtostrf(now.second(), 2, 0, rsec);
    record = (utime + comma + ryear + slash + rmonth + slash + rday + space + rhour + colon + rmin + colon + rsec + comma +
              xAxisRawData + comma + yAxisRawData + comma + zAxisRawData + comma +
              temp + comma + press + comma + alt + spress);
    File dataFile = SD.open("LOG.csv", FILE_WRITE);
    if (dataFile)
    
    {
      dataFile.println(record);
      dataFile.close();
      Serial.println(record);
    }
    else
    {
      Serial.println("error opening LOG.csv");
    }
  #endif
  
  delay(1000);
};
