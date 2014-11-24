/*
* SHT21.h header file for SHT21 library for Energia
* 
* Author: 	ArcticSaturn
* Date:		Nov 2014
* Rev:		1.0
*
*/
#ifndef SHT21_h
#define SHT21_h

#include "Energia.h"
#include <SoftwareI2C.h>


/*
* General defines
*/
#define START_T_MEAS_HM		0xE3 	// start temperature meas. hold master
#define START_RH_MEAS_HM	0xE5 	// start humidity meas. hold master
#define START_T_MEAS_POLL	0xF3 	// start temperature meas. no hold master
#define START_RH_MEAS_POLL	0xF5 	// start humidity meas. no hold master
#define USER_REG_W		0xE6	// command writing user register
#define USER_REG_R		0xE7	// command reading user register
#define SOFT_RESET		0xFE	// command soft reset

#define RES_12_14BIT		0x00	// RH=12bit, T=14bit
#define RES_8_12BIT		0x01	// RH= 8bit, T=12bit
#define RES_10_13BIT		0x80	// RH=10bit, T=13bit
#define RES_11_11BIT		0x81	// RH=11bit, T=11bit

#define RH_8BIT			0x01	// RH=8bit,  was 0x01
#define RH_10BIT		0x10	// RH=10bit, was 0x80
#define RH_11BIT		0x11	// RH=11bit, was 0x81
#define RH_12BIT		0x00	// RH=12bit, was 0x00

#define T_11BIT			0x81	// T=11bit
#define T_12BIT			0x01	// T=12bit
#define T_13BIT			0x10	// T=13bit
#define T_14BIT			0x00	// T=14bit

#define RES_MASK		0x81	// Mask for res. bits (7,0) in user reg.

#define EOB_O			0x40	// end of battery
#define EOB_MASK		0x40	// Mask for EOB bit(6) in user reg.

#define HEATER_ON		0x04	// heater on
#define HEATER_OFF		0x00	// heater off
#define HEATER_MASK		0x04	// Mask for Heater bit(2) in user reg.

#define I2C_ADDR_W		0x80	// I2C address for write access
#define I2C_ADDR_R		0x81	// I2C address for read access

/*
* measurement signal selection
*/
typedef enum{
HUMIDITY,
TEMPERATURE
}enmMeasureType;

class SHT21
{
  public:
	SHT21(uint8_t pinSDA, uint8_t pinSCL);
	void SoftReset();
	void MeasurePolling(enmMeasureType MeasureType, uint16_t *pRawMeasValue, float *pMeasValue);
	void Measure(enmMeasureType MeasureType, uint8_t Resolution, uint16_t *pRawMeasValue, float *pMeasValue);
	
  //private:
	float CalcTemperature(uint16_t rawTemperature);
	float CalcHumidity(uint16_t rawHumidity);
	void ReadUserRegister(uint8_t *pRegisterValue);
	void WriteUserRegister(uint8_t pRegisterValue);
};

#endif
