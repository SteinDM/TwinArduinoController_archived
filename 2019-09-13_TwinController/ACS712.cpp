#include "ACS712.h"
int calibratedVoltage; 
float vref;
float offsetADCpoints;

ACS712::ACS712(ACS712_type type, uint8_t _pin) {
	switch (type) {
		case ACS712_05B:
			sensitivity = 185;
			break;
		case ACS712_20A:
			sensitivity = 100;
			break;
		case ACS712_30A:
			sensitivity = 66;
			break;
		default:
			sensitivity = 66;
			break;
	}
	pin = _pin;
}

void ACS712::setSensitivity(float sens) {
	sensitivity = sens;
}

float ACS712::calibrate() {
	int _zero = 0;
	int samplesCount= 20;
	for (int i = 0; i < samplesCount; i++) {
		_zero += analogRead(pin);
		delay(10);
	}
	offsetADCpoints = (ADC_SCALE*samplesCount/2-_zero);
	offsetADCpoints /= samplesCount;
	return offsetADCpoints;
}

long ACS712::getCallibratedVoltage(){
	return calibratedVoltage;
}


float ACS712::getCurrentDC() {
	float I = (((ADC_SCALE/2)-(analogRead(pin)+0.25)-offsetADCpoints)*vref/ADC_SCALE)/sensitivity; //http://www.skillbank.co.uk/arduino/measure.htm explains why +0.25
	return I;
}

long ACS712::setVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCW;
  // result = ADCL;
  // result |= ADCH<<8;
  vref=1126400L / (result+0.5); // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000 (see http://www.skillbank.co.uk/arduino/measure.htm -> used 1.1*1024*1000 and (result +0.5)
  calibratedVoltage=1125300L/((result-offsetADCpoints)*2);
  
  return result;
}
