#include <Arduino.h>
#include "Ewma.h"
#include "ACS712.h"
#include <LiquidCrystal.h>
#include <AutoPID.h>

////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// initialize variables

int i;
int enableMotorSignal=0;
unsigned long previousMillisLCD = 0;           // will store last time LCD was updated
unsigned long previousMillisIMotor = 0;           // will store last time motor current was measured
unsigned long previousMillisIGenerator=0;
unsigned long lastThrottleUpdate=0;
const long lCDWriteInterval = 1000;        // interval at which to write lcd
const long currentMotorMeasureInterval = 10;    // interval at which to measure current
const long currentGeneratorMeasureInterval = 10;    // interval at which to measure current
const long throttelUpdateInterval=100;
float iSmoothMotor=0;
double pSmoothMotor=0;
float iSmoothGenerator=0;
double pSmoothGenerator=0;
float batteryVoltage=50;

unsigned int ADCValue;
double Voltage;
long vcc = 5000;
float offsetGenerator =0 ;
float offsetMotor = 0;
unsigned long currentMilliSeconds;

int enable_Pin = 6;

////////////////////////////////////////////////////////////////////////////
// initialize perhipicals
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);	//lcd screen

ACS712 generatorCurrentSensor(ACS712_30A,A1);
ACS712 motorCurrentSensor(ACS712_30A,A0);

////////////////////////////////////////////////////////////////////////////
//pid settings and gains		// 1-4.2v on kt, 1.2-4.3v on green earth, 1.1-3.8 on ebike
#define OUTPUT_MIN 51 		//(1v-> 255/5*1)
#define OUTPUT_MAX 214 		//(4.2v? ->255/5*4.2=214 )
#define KP 0.12//.12				// proportional
#define KI 0.2 //.0003			// Integral
#define KD 0.0001

////////////////////////////////////////////////////////////////////////////
//


////////////////////////////////////////////////////////////////////////////
//Initializes Filters and regulators
Ewma adcFilterGenerator(0.01);		// smoothing of input signals
Ewma adcFilterMotor(0.1);			//
double motorPower = 0, assistPowerSetpoint = 100, pwmThrottleOutput =0, motorPowerSetpoint = 0;
AutoPID throttlePID(&pSmoothMotor, &motorPowerSetpoint, &pwmThrottleOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);	//Pid controller here//input/output variables passed by reference, so they are updated automatically


////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {  // measure current
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
//  delay(500);
  pinMode(enable_Pin, INPUT); 
  offsetGenerator = generatorCurrentSensor.calibrate(); // this sets the sensors offset.
  offsetMotor = motorCurrentSensor.calibrate();
  
  //throttlePID.setBangBang(50);
  throttlePID.setTimeStep(1000);
}

////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void loop() {
  /////////////////////////////////////////////////////////////////////////
	currentMilliSeconds = millis(); // need this to do proper waits.
	

///Measure generator current and smooth out, calculate power
	if (currentMilliSeconds - previousMillisIGenerator  >= currentGeneratorMeasureInterval){
		vcc = generatorCurrentSensor.setVcc();
		float Igen = generatorCurrentSensor.getCurrentDC();
		iSmoothGenerator = adcFilterGenerator.filter(Igen);
		pSmoothGenerator=batteryVoltage*iSmoothGenerator;
		previousMillisIGenerator=currentMilliSeconds;
		}

///Measure motor current and smooth out, calculate power
	if (currentMilliSeconds - previousMillisIMotor >= currentMotorMeasureInterval){
		vcc = motorCurrentSensor.setVcc();
		float Imotor = motorCurrentSensor.getCurrentDC();
		iSmoothMotor = adcFilterMotor.filter(Imotor);
		pSmoothMotor = batteryVoltage*iSmoothMotor;
		previousMillisIMotor=currentMilliSeconds;
		}

// do pid caluclation and set pwm duty cycle


	if (digitalRead(enable_Pin))
	{

		motorPowerSetpoint = assistPowerSetpoint + 0 +200;
		throttlePID.run();
	}
	else{
		pwmThrottleOutput=0;
		throttlePID.reset();
	}



//Write stuff to lcd
	if (currentMilliSeconds - previousMillisLCD>= lCDWriteInterval){

		lcd.setCursor(0,0);
		lcd.print(String("I="));
		if (iSmoothMotor>=0){
		  lcd.print(" ");
		  }
		lcd.print(iSmoothMotor);
		lcd.print( "A");


		lcd.setCursor(0, 1);
		lcd.print("Pwm=");
		if (iSmoothMotor>=0){
		  lcd.print(" ");
		  }
		lcd.print(pwmThrottleOutput);
		lcd.print(" ");

		previousMillisLCD=currentMilliSeconds;
		}
}
