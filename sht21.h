/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef SHT21_h
#define SHT21_h

#include "Energia.h"
#include <SoftwareI2C.h>



// measurement signal selection
typedef enum{
HUMIDITY,
TEMP
}enmMeasureType;

class SHT21
{
  public:
	SHT21(uint8_t pinSDA, uint8_t pinSCL);
    uint8_t SoftReset();
	void MeasurePolling(enmMeasureType MeasureType, uint16_t* MeasValue, float* MeasValue2);
	
  private:
    int _pin;
	float CalcRH(uint16_t rawRH);
	float CalcTemperature(uint16_t rawTemperature);
	
};

#endif
