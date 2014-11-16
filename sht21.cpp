/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Energia.h"
#include "sht21.h"

SWI2C myI2C;    // instantiate softwareI2C class


SHT21::SHT21(uint8_t pinSDA, uint8_t pinSCL)
{
  myI2C.setI2Cpins(pinSDA,pinSCL);
  myI2C.begin();
  
}

uint8_t SHT21::SoftReset()
{


}


// INPUT: 	MeasureType = HUMIDITY, TEMPERATURE
//			MeasValue = address pointer to variable
void SHT21::MeasurePolling(enmMeasureType MeasureType, uint16_t* MeasValue, float* MeasValue2)
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
	
	*MeasValue=(uint16_t)data1*256;
	*MeasValue+=data2;
	*MeasValue2=CalcTemperature(MeasValue);
}


float SHT21::CalcTemperature(uint16_t* rawTemperature)
{
	float result;
	result=-46.85 + 175.72*(*rawTemperature)/65536;
}