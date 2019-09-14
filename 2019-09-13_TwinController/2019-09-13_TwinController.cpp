#include <Arduino.h>
#include "Ewma.h"
#include "ACS712.h"
#include <LiquidCrystal.h>
#include <AutoPID.h>

////////////////////////////////////////////////////////////////////////////
// initialize the instances with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
ACS712 sensor(ACS712_30A,A0);
Ewma adcFilter1(0.01);
int i;
unsigned long previousMillisLCD = 0;           // will store last time LCD was updated
unsigned long previousMillisI = 0;           // will store last time LCD was updated
const long lCDWriteInterval = 1000;        // interval at which to write lcd
const long currentMeasureInterval = 10;    // interval at wich to measure current
float Iavg=0;

unsigned int ADCValue;
double Voltage;
long vcc = 5000;
float offset =0 ;




void setup() {  // measure current
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
//  lcd.print("Current measures");
  delay(500);
  offset = sensor.calibrate(); // this sets the sensors offset.
}

void loop() {
  /////////////////////////////////////////////////////////////////////////
  // write screen

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisI >= currentMeasureInterval){
    vcc = sensor.setVcc();
    float I = sensor.getCurrentDC();
    Iavg = adcFilter1.filter(I);
    previousMillisI=currentMillis;
    }

    if (currentMillis - previousMillisLCD>= lCDWriteInterval){

      lcd.setCursor(0,0);
      lcd.print(String("I="));
      if (Iavg>=0){
		  lcd.print(" ");
		  }
      lcd.print(Iavg);
      lcd.print( "A");


      lcd.setCursor(0, 1);
      lcd.print("P="); //vcc);
      if (Iavg>=0){
		  lcd.print(" ");
		  }
      lcd.print(48*Iavg);
      lcd.print("W");

      previousMillisLCD=currentMillis;
    }
}
