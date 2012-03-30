#include <Wire.h>

//sensor
#include <Adafruit_BMP085.h>

//lcd display
#include <LiquidCrystal.h>
#include <Math.h>

//sd card
#include "RTClib.h"
#include <SD.h>

#define LOG_INTERVAL 1000
#define SYNC_INTERVAL 1000
#define ECHO_TO_SERIAL
#define redLEDpin 2
#define greenLEDpin 3
RTC_DS1307 RTC;

const int chipSelect = 10;

//from adafruit github
Adafruit_BMP085 bmp;

//lcd pins
//LiquidCrystal lcd(RS, EN, DB4, DB5, DB6, DB7);
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//array of degree symbol for lcd
byte degrSym[8] = {
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

//BMP085 declare variables
float tempC;
float tempF;
int presPA;
float presInHg;
int meters;
float feet;

//function to convert Celsius to Fahrenheit
float cToF(float c) {
  return ((c * 9.0) / 5.0) + 32.0;
}

//function to convert Pa to inches Hg
float paToInHg(float pa) {
  return pa * 0.000295333727;
}

//function to convert meters to feet
float mToFt(int m) {
  return m * 3.2808399;
}

void error(char *str) {
  Serial.print("error: ");
  Serial.println(str);
  //turn on warning light on logger
  digitalWrite(redLEDpin, HIGH);
  while(1);
}

void setup() {
  Serial.begin(9600);
//datalogger
  Serial.print("Initializing SD card...");
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(redLEDpin, LOW);
  digitalWrite(greenLEDpin, LOW);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

//BMP085 start
  bmp.begin();

//LCD start
  lcd.createChar(0, degrSym);
  lcd.begin(16, 2);
}

void loop() {
    DateTime now;
    delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
    
    digitalWrite(greenLEDpin, HIGH);
    //grab curent time from realtime clock
    now = RTC.now();
    
  //initialize BMP085 variables
    float tempC = 0;
    int presPa = 0;
    int meters = 0;
    float tempf = 0.0;

  //read sensor values into variables
    tempC = bmp.readTemperature();
    tempF = floor(cToF(tempC) * 10 + 0.5) / 10;
    presPa = bmp.readPressure();
    presInHg = paToInHg(presPa);
    meters = bmp.readAltitude(101760);
    feet = mToFt(meters);

  //ouput to serial
    Serial.print("Temperature = ");
    Serial.print(tempC);
    Serial.println(" *C");
    Serial.print(tempF);
    Serial.println(" *F");
    Serial.print("Pressure = ");
    Serial.print(presPa);
    Serial.println(" Pa");
    Serial.println();

  //output to lcd screen
    lcd.setCursor(0,0);
    lcd.print(tempF,1);
    lcd.write(byte(0));
    lcd.print("F");
    lcd.setCursor(8,0);
    lcd.print(paToInHg(bmp.readPressure()));
    lcd.print(" Hg");
    if (feet <= 9)
      lcd.setCursor(12,1);
      lcd.print(feet);
      lcd.print(" ft");
    if (feet <= 99 && feet > 9)
      lcd.setCursor(11,1);
      lcd.print(feet);
      lcd.print(" ft");
    if (feet <=999 && feet > 99)
      lcd.setCursor(10,1);
      lcd.print(feet);
      lcd.print(" ft");
    if (feet <=9999 && feet > 999)
      lcd.setCursor(9,1);
      lcd.print("Seek O2");

// log to sd card
  File dataFile = SD.open("LOG.CSV", FILE_WRITE);
  
  if (dataFile) {
    dataFile.print(now.unixtime());
    dataFile.print(",");
    dataFile.print(now.year(), DEC);
    dataFile.print("/");
    dataFile.print(now.month(), DEC);
    dataFile.print("/");
    dataFile.print(now.day(), DEC);
    dataFile.print(" ");
    dataFile.print(now.hour(), DEC);
    dataFile.print(":");
    dataFile.print(now.minute(), DEC);
    dataFile.print(":");
    dataFile.print(now.second(), DEC);
    dataFile.print(",");
    dataFile.print(tempC);
    dataFile.print(",");
    dataFile.print(tempF);
    dataFile.print(",");
    dataFile.print(bmp.readPressure());
    dataFile.print(",");
    dataFile.print(paToInHg(bmp.readPressure()));
    dataFile.print(",");
    dataFile.print(meters);
    dataFile.print(",");
    dataFile.println(feet);
    dataFile.close();
    
    digitalWrite(greenLEDpin, LOW);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}
