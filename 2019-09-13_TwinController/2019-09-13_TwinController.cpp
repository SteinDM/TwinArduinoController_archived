#include <Arduino.h>
#include "Ewma.h"
#include "ACS712.h"
#include <LiquidCrystal.h>
//#include <AltSoftSerial.h>
//#include <SoftwareSerial.h>
#include "AutoPID.h"
#include "ReceiveOnlySoftwareSerial.h"



////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// initialize variables
ReceiveOnlySoftwareSerial  displaySerialRx(10); // RX Not all pins on the Leonardo and Micro support change interrupts, so only the following can be used for RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
//ReceiveOnlySoftwareSerial motorSerialRx(8);
uint8_t ui8_rx_bufferDisplay [16];
//uint8_t ui8_rx_bufferMotor[11];


bool generatorRunning=0;						//set generator signal inactive while starting up
bool brakeActiveSignal=1;							//set brake active while starting up

unsigned long previousMillisUphillAssist = 0;       // will store last time the uphill assist was updated
unsigned long previousMillisLCD = 0;           		// will store last time LCD was updated
unsigned long previousMillisIMotor = 0;           	// will store last time motor current was measured
unsigned long previousMillisIGenerator=0;
unsigned long lastThrottleUpdate=0;
const long lCDWriteInterval = 300;        			// interval at which to write lcd
const long currentMotorMeasureInterval = 10;    	// interval at which to measure current
const long currentGeneratorMeasureInterval = 10;    // interval at which to measure current
const long throttelUpdateInterval=20;
const long uphillAssistUpdateInterval = 5;   	 	// interval at which update the Uphill assist level ms - if 5 it means 5 seconds before full uphill assist level (1000w) and so on...

//variables current sensors
float iSmoothMotor=0;
double pSmoothMotor=0;
float iSmoothGenerator=0;
double pSmoothGenerator=0;

//variables display
float batteryVoltage=40;
uint8_t ui8_startBitDisplaySerial = 0;
uint8_t ui8_light_On = 0;
uint8_t ui8_WalkModus_On = 0;
uint8_t ui8_assist_level = 0;
uint8_t ui8_max_speed = 0;
uint8_t ui8_wheel_size = 0;

/*
//variables controller
uint8_t ui8_motorStartByte;
uint8_t ui8_battery_soc;
uint8_t ui8_nominalBatteryVoltage;
uint16_t ui16_wheel_period_ms;
uint8_t ui8_crcBit;
uint8_t ui8_MovingModeIndication;
uint8_t ui8_BatteryCurrent;  //4x controller current
*/

unsigned long currentMilliSeconds;

int brake_DI_Pin = 7;
int generatorEnable_DI_Pin = 8;
int throttlePwm_Pin= 9; // pin 3, 5, 6, 9, 10 11 support pwm

////////////////////////////////////////////////////////////////////////////
// initialize perhipicals
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);	//lcd screen

ACS712 generatorCurrentSensor(ACS712_30A,A0);
ACS712 motorCurrentSensor(ACS712_30A,A1);

////////////////////////////////////////////////////////////////////////////
//pid settings and gains		// 1-4.2v on kt, 1.2-4.3v on green earth, 1.1-3.8 on ebike
#define PWM_OUTPUT_MIN 65 		//(1v-> 255/5*1=51)
#define PWM_OUTPUT_MAX 214 		//(4.2v? ->255/5*4.2=214 )
#define KP 0.12//.12				// proportional
#define KI 0.2 //.0003			// Integral
#define KD 0.0001

////////////////////////////////////////////////////////////////////////////
//


////////////////////////////////////////////////////////////////////////////
//Initializes Filters and regulators
Ewma adcFilterGenerator(0.01);		// smoothing of input signals
Ewma adcFilterMotor(0.1);			// higher means faster but more likely to oscillate- 0.1 seems to be the highest value not resulting in oscillations when pedaqling and measuring every 10ms
double motorPower = 0, pwmThrottleOutput =0, motorPowerSetpoint = 0;
int16_t uphillAssistPower = 0;
AutoPID throttlePID(&pSmoothMotor, &motorPowerSetpoint, &pwmThrottleOutput, PWM_OUTPUT_MIN, PWM_OUTPUT_MAX, KP, KI, KD);	//Pid controller here//input/output variables passed by reference, so they are updated automatically


////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {  // measure current
	displaySerialRx.begin(9600);
	displaySerialRx.setTimeout(10);
//	motorSerialRx.begin(9600);
//	motorSerialRx.end();
//	motorSerialRx.setTimeout(10);
//	Serial.begin(9600);

	lcd.begin(16, 2);
	pinMode(generatorEnable_DI_Pin, INPUT);
	pinMode(throttlePwm_Pin, OUTPUT);
	generatorCurrentSensor.calibrate(); // this sets the sensors offsets
	motorCurrentSensor.calibrate();
//	throttlePID.setBangBang(OUTPUT_MIN,PWM_OUTPUT_MAX);
	throttlePID.setTimeStep(throttelUpdateInterval);
	throttlePID.reset();


}

////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void loop() {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///Read all variables for propper speed regulation/////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// read power from display trough serial connection
if (displaySerialRx.available()){
	  //firs bit seems to always be 16 as a checksum thing
	displaySerialRx.readBytesUntil('x',ui8_rx_bufferDisplay,16);
	ui8_startBitDisplaySerial = ui8_rx_bufferDisplay [0];
	if (ui8_startBitDisplaySerial==16){
		//ui8_light_On = ui8_rx_bufferDisplay [1] >> 7;
		//ui8_WalkModus_On = (ui8_rx_bufferDisplay [1] & 7)==6;
		if (ui8_assist_level != ui8_rx_bufferDisplay [1] & 7){
			ui8_assist_level = (ui8_rx_bufferDisplay [1] & 7);
			}
		//ui8_max_speed = 10 + ((ui8_rx_bufferDisplay [2] & 248) >> 3) | (ui8_rx_bufferDisplay [4] & 32);
		//ui8_wheel_size = ((ui8_rx_bufferDisplay [4] & 192) >> 6) | ((ui8_rx_bufferDisplay [2] & 7) << 2);
	}
}
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
//Measure time so all functions run at the proper time intervall
	currentMilliSeconds = millis();

//Set generator started bit if the generator started pin is high - (means someone is peddalling)
	generatorRunning = digitalRead(generatorEnable_DI_Pin);
	
//Set brake bit if the brake is pulled - (Todo; should this be an interup?)
	brakeActiveSignal = digitalRead(brake_DI_Pin);

//Manage pwm states
	
	

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///Do motor regulation /////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Case 1 - standstill and motor disabled (cover 3 states)
	if (((!brakeActiveSignal||brakeActiveSignal)&&!generatorRunning&&!pwmThrottleOutput)||(brakeActiveSignal&&generatorRunning&&!pwmThrottleOutput)){
		// three cases where motor is disabled - brake set and generator running - generator not running
		motorPowerSetpoint=0;
		pwmThrottleOutput=0;
		throttlePID.reset();
	}
// Case 2 - motor starts running
	else if (!brakeActiveSignal&&generatorRunning&&!pwmThrottleOutput){
		//brakes disengaged and generator is running -> start the motor
		if (pwmThrottleOutput<PWM_OUTPUT_MIN) {pwmThrottleOutput=PWM_OUTPUT_MIN;}
		if (uphillAssistPower>0)uphillAssistPower=0;

		motorPowerSetpoint = (pSmoothGenerator*ui8_assist_level)+pSmoothGenerator + uphillAssistPower; // assist will equal the generated power * the assist level on the steering wheel + the uphill aid
		throttlePID.run();
	}

// Case 3 - motor is running normally
	else if (!brakeActiveSignal&&generatorRunning&&pwmThrottleOutput){
		//Calculate uphill power assist level
		if ((currentMilliSeconds - previousMillisUphillAssist)>= uphillAssistUpdateInterval){
			if ((pwmThrottleOutput<PWM_OUTPUT_MIN+50)&&uphillAssistPower<1000) {uphillAssistPower++;}
			if ((pwmThrottleOutput<PWM_OUTPUT_MIN+10)&&uphillAssistPower>800) {uphillAssistPower--;}
			if (pwmThrottleOutput>PWM_OUTPUT_MIN+150&&uphillAssistPower>0)uphillAssistPower--;
		}
		// set the motor power
		motorPowerSetpoint = (pSmoothGenerator*ui8_assist_level)+pSmoothGenerator + uphillAssistPower; // assist will equal the generated power * the assist level on the steering wheel + the uphill aid
		throttlePID.run();
	}
// Case 4 - motor is enabled and brake is set
	else if ((!brakeActiveSignal&&(generatorRunning||!generatorRunning)&&pwmThrottleOutput)){
		if (pwmThrottleOutput>PWM_OUTPUT_MIN){
		motorPowerSetpoint = -(500*(pwmThrottleOutput-PWM_OUTPUT_MIN))/(PWM_OUTPUT_MAX-PWM_OUTPUT_MIN); //...500... watt max, ramping down linearly - means constant torque
		throttlePID.run();
		}
		else {  // motor stopped
			motorPowerSetpoint=0;
			pwmThrottleOutput=0;
			throttlePID.reset();
		}
		uphillAssistPower=0; // reset uphill asist power
	}
//Case 5 - motor is enabled and we're coasting
	else if (!brakeActiveSignal&&!generatorRunning&&pwmThrottleOutput){
		if (pwmThrottleOutput>PWM_OUTPUT_MIN){
		motorPowerSetpoint = -(50*(pwmThrottleOutput-PWM_OUTPUT_MIN))/(PWM_OUTPUT_MAX-PWM_OUTPUT_MIN); //...50... watt coast brake, ramping down linearly - means constant torque
		throttlePID.run();
		}
		else {  // motor stopped
			motorPowerSetpoint=0;
			pwmThrottleOutput=0;
			throttlePID.reset();
		}
		uphillAssistPower=0; // reset uphill asist power
	}


analogWrite(throttlePwm_Pin,pwmThrottleOutput);

//Write stuff to lcd
	if (currentMilliSeconds - previousMillisLCD>= lCDWriteInterval){

		lcd.setCursor(0,0);
		/*
		lcd.setCursor(10,0);
		lcd.print("En=");
		lcd.print(digitalRead(enable_Pin));
		*/
		lcd.print("B");
		lcd.print(brakeActiveSignal);
		lcd.print(" ");
		lcd.print("G");
		lcd.print(generatorRunning);
		lcd.print(" ");
		lcd.print("A");
		lcd.print(ui8_assist_level);
		lcd.print(" ");
		lcd.print(String("Im="));
				if (iSmoothMotor>=0){
				  lcd.print(" ");
				  }
		lcd.print(iSmoothMotor,1);
		lcd.print( "A");

		lcd.setCursor(0, 1);
		lcd.print("Ig=");
		if (iSmoothGenerator>=0){
		  lcd.print(" ");
		  }
		lcd.print(iSmoothGenerator,1);
		lcd.print(" ");
		lcd.print("A");
		lcd.setCursor(9,1);
		lcd.print("pwm=");
		if (pwmThrottleOutput<10) lcd.print(" ");
		if (pwmThrottleOutput<100) lcd.print(" ");
		lcd.print(pwmThrottleOutput,0);
		previousMillisLCD=currentMilliSeconds;
		}
}


/*
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///Manage pwm states /////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pwmReset(){
	
}

*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///code recycle bin /////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//read motor stats from controller trough second serial connection
		//First bit seems to always be 65 as a checksum thing
/*	  if (motorSerialRx.available()){
	  //first bit seems to always be 16 as a checksum thing
	  motorSerialRx.readBytesUntil('x',ui8_rx_bufferMotor,11);

	ui8_motorStartByte = ui8_rx_bufferMotor [0];
	ui8_battery_soc = ui8_rx_bufferMotor [1];
	ui8_nominalBatteryVoltage = ui8_rx_bufferMotor [2];
	ui16_wheel_period_ms=ui8_rx_bufferMotor [3]<<8+ui8_rx_bufferMotor [4];//			ui8_tx_buffer [3] = (ui16_wheel_period_ms >> 8) & 0xff;			ui8_tx_buffer [4] = ui16_wheel_period_ms & 0xff;
	ui8_crcBit=ui8_rx_bufferMotor [6];
	ui8_MovingModeIndication = ui8_rx_bufferMotor [7];
	ui8_BatteryCurrent= ui8_rx_bufferMotor [8];  //4x controller current
  }
*/
/*
	if (digitalRead(generatorEnable_DI_Pin))
	{
		motorPowerSetpoint = uphillAssistPower + pSmoothGenerator*ui8_assist_level;
		throttlePID.run();
	}
	else {
		pwmThrottleOutput=0;
		throttlePID.reset();
	}

			uint8_t x=0;
		lcd.print(brakeActiveSignal);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [2+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [3+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [4+x]);
		lcd.print(" ");
		lcd.print(ui8_rx_bufferDisplay [ui8_assist_level]);
		lcd.print(" ");
*/
