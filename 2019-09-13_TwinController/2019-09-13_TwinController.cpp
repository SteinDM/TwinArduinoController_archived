#include <Arduino.h>
#include "Ewma.h"
#include "ACS712.h"
#include <LiquidCrystal.h>
#include "AutoPID.h"
//#include "AutoPID.cpp"

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
const long lCDWriteInterval = 300;        // interval at which to write lcd
const long currentMotorMeasureInterval = 10;    // interval at which to measure current
const long currentGeneratorMeasureInterval = 10;    // interval at which to measure current
const long throttelUpdateInterval=20;
float iSmoothMotor=0;
double pSmoothMotor=0;
float iSmoothGenerator=0;
double pSmoothGenerator=0;
float batteryVoltage=40;

unsigned long currentMilliSeconds;

int brake_Pin = 7;
int enable_Pin = 8;
int throttlePwm_Pin= 9; // pin 3, 5, 6, 9, 10 11 support pwm

////////////////////////////////////////////////////////////////////////////
// initialize perhipicals
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);	//lcd screen

ACS712 generatorCurrentSensor(ACS712_30A,A0);
ACS712 motorCurrentSensor(ACS712_30A,A1);

////////////////////////////////////////////////////////////////////////////
//pid settings and gains		// 1-4.2v on kt, 1.2-4.3v on green earth, 1.1-3.8 on ebike
#define OUTPUT_MIN 65 		//(1v-> 255/5*1=51)
#define OUTPUT_MAX 214 		//(4.2v? ->255/5*4.2=214 )
#define KP 0.12//.12				// proportional
#define KI 0.2 //.0003			// Integral
#define KD 0.0001

////////////////////////////////////////////////////////////////////////////
//


////////////////////////////////////////////////////////////////////////////
//Initializes Filters and regulators
Ewma adcFilterGenerator(0.01);		// smoothing of input signals
Ewma adcFilterMotor(0.1);			// higher means faster but more likely to oscillate- 0.1 seems to be the highest value not resulting in oscillations when pedaqling and measuring every 10ms
double motorPower = 0, assistPowerSetpoint = 0, pwmThrottleOutput =0, motorPowerSetpoint = 0;
AutoPID throttlePID(&pSmoothMotor, &motorPowerSetpoint, &pwmThrottleOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);	//Pid controller here//input/output variables passed by reference, so they are updated automatically


////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {  // measure current

  lcd.begin(16, 2);
  pinMode(enable_Pin, INPUT); 
  pinMode(throttlePwm_Pin, OUTPUT);
  generatorCurrentSensor.calibrate(); // this sets the sensors offset.
  motorCurrentSensor.calibrate();
  
  throttlePID.setBangBang(OUTPUT_MIN,OUTPUT_MAX);
  throttlePID.setTimeStep(throttelUpdateInterval);
  throttlePID.reset();
}

////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void loop() {
  /////////////////////////////////////////////////////////////////////////
	currentMilliSeconds = millis(); // need this to do proper waits.
	

///Measure generator current and smooth out, calculate power
	if (currentMilliSeconds - previousMillisIGenerator  >= currentGeneratorMeasureInterval){
		generatorCurrentSensor.setVcc();
		float Igen = generatorCurrentSensor.getCurrentDC();
		iSmoothGenerator = adcFilterGenerator.filter(Igen);
		pSmoothGenerator=batteryVoltage*iSmoothGenerator;
		previousMillisIGenerator=currentMilliSeconds;
		}

///Measure motor current and smooth out, calculate power
	if (currentMilliSeconds - previousMillisIMotor >= currentMotorMeasureInterval){
		motorCurrentSensor.setVcc();
		float Imotor = motorCurrentSensor.getCurrentDC();
		iSmoothMotor = adcFilterMotor.filter(Imotor);
		pSmoothMotor = -batteryVoltage*iSmoothMotor;
		previousMillisIMotor=currentMilliSeconds;
		}

// do pid caluclation and set pwm duty cycle


	if (digitalRead(enable_Pin))
	{
		if (pwmThrottleOutput<OUTPUT_MIN) {pwmThrottleOutput=OUTPUT_MIN;}
		motorPowerSetpoint = assistPowerSetpoint + pSmoothGenerator;
		throttlePID.run();
	}
	else if (pwmThrottleOutput!=0){
		pwmThrottleOutput=0;
		throttlePID.reset();
	}

	analogWrite(throttlePwm_Pin,pwmThrottleOutput);



//Write stuff to lcd
	if (currentMilliSeconds - previousMillisLCD>= lCDWriteInterval){

		lcd.setCursor(0,0);
		lcd.print(String("Im="));
		if (iSmoothMotor>=0){
		  lcd.print(" ");
		  }
		lcd.print(iSmoothMotor);
		lcd.print( "A");
		lcd.setCursor(10,0);
		lcd.print("En=");
		lcd.print(digitalRead(enable_Pin));


		lcd.setCursor(0, 1);
		lcd.print("Ig=");
		if (iSmoothGenerator>=0){
		  lcd.print(" ");
		  }
		lcd.print(iSmoothGenerator);
		lcd.print(" ");
		lcd.print("A");
		lcd.setCursor(9,1);
		lcd.print("pwm=");
		lcd.print(pwmThrottleOutput);




		previousMillisLCD=currentMilliSeconds;
		}
}
