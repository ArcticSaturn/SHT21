/*
* SHT21 library for Energia
*
* Author: 	ArcticSaturn
* Date:		Nov 2014
* Rev:		1.0
*
* open items:
* - implement hold master measurement
* - change calculation of temperature/humidity to integer calculation
    to save code size
*   result=-46.85 + 175.72*(float)rawTemperature/65536;
*   result = 21965*raw>>13 - 46850  21965= 175720/8
*/

#include "Energia.h"
#include "sht21.h"

SWI2C myI2C (5,6);    // instantiate local softwareI2C class
  

SHT21::SHT21(uint8_t pinSDA, uint8_t pinSCL)
{
	SWI2C myI2C (pinSDA,pinSCL);    // instantiate local softwareI2C class
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
	delayMicroseconds(15000);	// wait for reset to end
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
	//myI2C.i2cStop();
}

/*
 * Performs a measurement of SHT21
 * input:	MeasureType	TEMPERATURE or HUMIDITY
 *		Resolution	sets the desired resolution of the measurement
		*pRawMeasValue	pointer with measured values
		*pMeasValue	pointer with calculated value
 * output:	_MsgCorrect	0 if not correct
 * 				1 if correct
 */
void SHT21::Measure(enmMeasureType MeasureType, uint8_t Resolution, 
		uint16_t *pRawMeasValue)
{
	uint8_t _data1, _data2;
	uint8_t _UsrRegValue;
	
	// read user register and set resolution values
	ReadUserRegister(&_UsrRegValue);
	_UsrRegValue = (_UsrRegValue & ~RES_MASK) | Resolution;
	WriteUserRegister(_UsrRegValue);
	
	//WriteUserRegister(Resolution);  // work around for not working SHT21
	
	
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_W);
	// send measurement trigger depending on measurement type
	if(MeasureType == TEMPERATURE)
		myI2C.i2cWrite2(START_T_MEAS_POLL);   // trigger Temp measure; no hold master
	else
		myI2C.i2cWrite2(START_RH_MEAS_POLL);   // trigger RH measure; no hold master
	
	//delay(100);
	myI2C.i2cStart();
	myI2C.i2cWrite(I2C_ADDR_R);
	
	
	// polling until slave send acknowledge bit
	do {
		delayMicroseconds(10000);
		myI2C.i2cStart();       
	}while(	myI2C.i2cWrite(I2C_ADDR_R) == NO_I2C_ACK ); 
		
	
	// read measurement value
	_data1 = myI2C.i2cRead(1);   // get meas value MSByte
	_data2 = myI2C.i2cRead(1);   // get meas value LSByte
	myI2C.i2cRead(0);           // get CRC8 value
	myI2C.i2cStop();        // end I2C transmission
	
	// conversion to uint16_t values
	*pRawMeasValue=(uint16_t)_data1*256;	// shift 8 times to left
	_data2&=0xFC;				// clear bit1/bit0 as these are status bits
	*pRawMeasValue+=_data2;	
	/*if(MeasureType == TEMPERATURE)
		*pMeasValue=CalcTemperature(*pRawMeasValue);
	else
		*pMeasValue=CalcHumidity(*pRawMeasValue);
	*/
}

long SHT21::CalcTemperature(uint16_t rawTemperature)
{
	//-46.85 + 175.72*(float)rawTemperature/65536;
	//21965= 175720/8
	long result;
	result = long(21965*(long)rawTemperature);
	result = result >> 13;
	result -= 46850;
	return result;

}

long SHT21::CalcHumidity(uint16_t rawHumidity)
{
	//-6+125*(float)rawHumidity/65536;
	//15625= 125000/8
	long result;
	result = long(15625*(long)rawHumidity);
	result = result >> 13;
	result -= 6000;
	return result;

}

float SHT21::CalcTemperature2(uint16_t rawTemperature)
{
	float result;
	result=-46.85 + 175.72*(float)rawTemperature/65536;
	return result;
}
float SHT21::CalcHumidity2(uint16_t rawHumidity)
{
	float result;
	result=-6+125*(float)rawHumidity/65536;
	return result;
}