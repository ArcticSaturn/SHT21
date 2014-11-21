/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Energia.h"
#include "sht21.h"


SWI2C myI2C (8,9);    // instantiate local softwareI2C class
  

SHT21::SHT21(uint8_t pinSDA, uint8_t pinSCL)
{
	//SWI2C myI2C (pinSDA,pinSCL);    // instantiate local softwareI2C class
	myI2C.setI2Cpins(pinSDA,pinSCL);
	myI2C.begin();
	
}

/*
* Perform soft reset of SHT21
* registers will be reset to default settings except for heater bit
* soft reset takes less than 15ms
*/
void SHT21::SoftReset()
{
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_W);
	myI2C.i2cWrite(SOFT_RESET);
	myI2C.i2cStop();
	delay(15);	// wait for reset to end
}


/*
* Read user register of SHT21
* input: pointer to result variable
* output: none, result will be written to pointer variable
*/
void SHT21::ReadUserRegister(uint8_t *pRegisterValue)
{
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_W);
	myI2C.i2cWrite(USER_REG_R);
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_R);
	*pRegisterValue = myI2C.i2cRead(1);	// get meas value MSByte
	myI2C.i2cRead(0);			// get CRC8 value
	myI2C.i2cStop();
}

/*
* Write user register of SHT21
* input: pointer to variable with new register content
* output: n/a
*/
void SHT21::WriteUserRegister(uint8_t pRegisterValue)
{
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_W);
	myI2C.i2cWrite(USER_REG_W);
	myI2C.i2cWrite(pRegisterValue);
	delayMicroseconds(10);
	myI2C.i2cStop();
}


// INPUT: 	MeasureType = HUMIDITY, TEMPERATURE
//			MeasValue = address pointer to variable
void SHT21::MeasurePolling(enmMeasureType MeasureType, uint16_t *pRawMeasValue, float *pMeasValue)
{
	uint8_t data1, data2;

	// ===========================================================
	//              SHT21 TEMPERATURE MEASUREMENT
	// ===========================================================
	myI2C.i2cStart();
	myI2C.i2cWrite(0x80);   // 0b10000000  sensor addres + write attempt
	myI2C.i2cWrite(0xE6);   // write register
	myI2C.i2cWrite(0x82);   // set resolution to 11bit
	myI2C.i2cStart();
	myI2C.i2cWrite(0x80);   // 0b10000000  sensor addres + write attempt
	myI2C.i2cWrite(0xF3);   // trigger temp measure; no hold master
	delay(43);              // wait for measurment end
	myI2C.i2cStart();       
	myI2C.i2cWrite(0x81);   // 0b10000000  sensor addres + read attempt
	data1 = myI2C.i2cRead(1);   // get meas value MSByte
	data2 = myI2C.i2cRead(1);   // get meas value LSByte
	myI2C.i2cRead(0);           // get CRC8 value
	myI2C.i2cStop();        // end I2C transmission
	
	*pRawMeasValue=(uint16_t)data1*256;
	*pRawMeasValue+=data2;
	*pMeasValue=CalcTemperature(*pRawMeasValue);
}

void SHT21::Measure(enmMeasureType MeasureType, uint8_t Resolution, 
		uint16_t *pRawMeasValue, float *pMeasValue)
{
	uint8_t _data1, _data2;
	uint8_t _UsrRegValue;
	
	// read user register and set resolution values
	ReadUserRegister(&_UsrRegValue);
	_UsrRegValue = (_UsrRegValue & ~RES_MASK) | Resolution;
	//WriteUserRegister(_UsrRegValue);
	WriteUserRegister(Resolution);
	
	
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_W);
	// send measurement trigger depending on measurement type
	if(MeasureType == TEMPERATURE)
		myI2C.i2cWrite2(START_T_MEAS_POLL);   // trigger Temp measure; no hold master
	else
		myI2C.i2cWrite2(START_RH_MEAS_POLL);   // trigger RH measure; no hold master
	
	/*delay(150);
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_R);
	*/
	// polling until slave send acknowledge bit
	do {
		delay(10);
		myI2C.i2cStart();       
	}while(	myI2C.i2cWrite(I2C_ADDR_R) == NO_I2C_ACK ); 
		
	
	// read measurement value
	_data1 = myI2C.i2cRead(1);   // get meas value MSByte
	_data2 = myI2C.i2cRead(1);   // get meas value LSByte
	myI2C.i2cRead(0);           // get CRC8 value
	myI2C.i2cStop();        // end I2C transmission
	
	// conversion to float values
	*pRawMeasValue=(uint16_t)_data1*256;	// shift 8 times to left
	_data2&=0xFC;				// clear bit1/bit0 as these are status bits
	*pRawMeasValue+=_data2;	
	if(MeasureType == TEMPERATURE)
		*pMeasValue=CalcTemperature(*pRawMeasValue);
	else
		*pMeasValue=CalcHumidity(*pRawMeasValue);
}

float SHT21::CalcTemperature(uint16_t rawTemperature)
{
	float result;
	result=-46.85 + 175.72*(float)rawTemperature/65536;
	return result;
}
float SHT21::CalcHumidity(uint16_t rawHumidity)
{
	float result;
	result=-6+125*(float)rawHumidity/65536;
	return result;
}