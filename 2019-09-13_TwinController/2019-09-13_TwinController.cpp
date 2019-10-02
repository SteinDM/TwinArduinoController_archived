#include <Arduino.h>
#include "Ewma.h"
#include "ACS712.h"
#include <LiquidCrystal.h>
//#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include "AutoPID.h"
#include "ReceiveOnlySoftwareSerial.h"


////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// initialize variables
ReceiveOnlySoftwareSerial  displaySerialRx(10); // RX Not all pins on the Leonardo and Micro support change interrupts, so only the following can be used for RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
ReceiveOnlySoftwareSerial motorSerialRx(8);
uint8_t ui8_rx_bufferDisplay [13];
uint8_t ui8_rx_bufferMotor[11];
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

//variables current sensors
float iSmoothMotor=0;
double pSmoothMotor=0;
float iSmoothGenerator=0;
double pSmoothGenerator=0;

//variables display
float batteryVoltage=40;
uint8_t ui8_light_On = 0;
uint8_t ui8_WalkModus_On = 0;
uint8_t ui8_assist_level = 0;
uint8_t ui8_max_speed = 0;
uint8_t ui8_wheel_size = 0;

//variables controller
uint8_t ui8_motorRxPackageChecksum;
uint8_t ui8_battery_soc;
uint8_t ui8_nominalBatteryVoltage;
uint16_t ui16_wheel_period_ms;
uint8_t ui8_crcBit;
uint8_t ui8_MovingModeIndication;
uint8_t ui8_BatteryCurrent;  //4x controller current


unsigned long currentMilliSeconds;

//int brake_Pin = 8;
int enable_Pin = 7;
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
	displaySerialRx.begin(9600);
	displaySerialRx.setTimeout(10);
	Serial.begin(9600);

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
// read power from display trough serial connection
	  if (displaySerialRx.available()){
		  //firs bit seems to always be 16 as a checksum thing
		displaySerialRx.readBytesUntil('x',ui8_rx_bufferDisplay,13);
	    Serial.write(ui8_rx_bufferDisplay[5]);
		ui8_light_On = ui8_rx_bufferDisplay [1] >> 7;
		ui8_WalkModus_On = (ui8_rx_bufferDisplay [1] & 7)==6;
		ui8_assist_level = ui8_rx_bufferDisplay [1] & 7;
		ui8_max_speed = 10 + ((ui8_rx_bufferDisplay [2] & 248) >> 3) | (ui8_rx_bufferDisplay [4] & 32);
		ui8_wheel_size = ((ui8_rx_bufferDisplay [4] & 192) >> 6) | ((ui8_rx_bufferDisplay [2] & 7) << 2);
	  }

//read motor stats from controller trough second serial connection
			//First bit seems to always be 65 as a checksum thing
		  if (motorSerialRx.available()){
			  //first bit seems to always be 16 as a checksum thing
			  motorSerialRx.readBytesUntil('x',ui8_rx_bufferMotor,11);
		    Serial.write(ui8_rx_bufferMotor[5]);

		    ui8_motorRxPackageChecksum = ui8_rx_bufferMotor [1];
		    ui8_battery_soc = ui8_rx_bufferMotor [1];
			ui8_nominalBatteryVoltage = ui8_rx_bufferMotor [2];
			ui16_wheel_period_ms=ui8_rx_bufferMotor [3]<<8+ui8_rx_bufferMotor [4];//			ui8_tx_buffer [3] = (ui16_wheel_period_ms >> 8) & 0xff;			ui8_tx_buffer [4] = ui16_wheel_period_ms & 0xff;
			ui8_crcBit=ui8_rx_bufferMotor [6];
			ui8_MovingModeIndication = ui8_rx_bufferMotor [7];
			ui8_BatteryCurrent= ui8_rx_bufferMotor [8];  //4x controller current
		  }



//Write stuff to lcd
	if (currentMilliSeconds - previousMillisLCD>= lCDWriteInterval){

		lcd.setCursor(0,0);
		/*lcd.print(String("Im="));
		if (iSmoothMotor>=0){
		  lcd.print(" ");
		  }
		lcd.print(iSmoothMotor);
		lcd.print( "A");
		lcd.setCursor(10,0);
		lcd.print("En=");
		lcd.print(digitalRead(enable_Pin));
		*/

		//lcd.print("abcd");
		//incomingChar="abcd";
		uint8_t x=0;
		lcd.print(ui8_rx_bufferDisplay [1+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [2+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [3+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [4+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [5+x]);
		lcd.print(" ");

		lcd.setCursor(0, 1);
		lcd.print(ui8_rx_bufferMotor [1+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferMotor [2+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferMotor [3+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferMotor [4+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferMotor [5+x]);
		lcd.print(" ");
		/*
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
*/



		previousMillisLCD=currentMilliSeconds;
		}
}
