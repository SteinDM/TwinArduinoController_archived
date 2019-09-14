#ifndef ACS712_h
#define ACS712_h

#include <Arduino.h>

#define ADC_SCALE 1024.0  // http://www.skillbank.co.uk/arduino/measure.htm explains why 1024
#define DEFAULT_FREQUENCY 50

enum ACS712_type {ACS712_05B, ACS712_20A, ACS712_30A};

class ACS712 {
public:
	ACS712(ACS712_type type, uint8_t _pin);
	float calibrate();
	void setVref(float _vref);
	void setSensitivity(float sens);
	float getCurrentDC();
	long setVcc();
	long getCallibratedVoltage();
	

private:
	float vref = 5000;  			 // this will be calibrated by the code
	float offsetADCpoints=0;		// this will be calibrated by the code
	float sensitivity;				
	uint8_t pin;
};

#endif
