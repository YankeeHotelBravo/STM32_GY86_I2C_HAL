#ifndef MS5611_H
#define MS5611_H

#include "main.h"




#define MS5611_ADDR 0x77

#define CONVERSION_OSR_256  1
#define CONVERSION_OSR_512  2
#define CONVERSION_OSR_1024 3
#define CONVERSION_OSR_2048 5
#define CONVERSION_OSR_4096 10

#define CMD_RESET 0x1E
#define CMD_PROM_C0 0xA0
#define CMD_PROM_C1 0xA2
#define CMD_PROM_C2 0xA4
#define CMD_PROM_C3 0xA6
#define CMD_PROM_C4 0xA8
#define CMD_PROM_C5 0xAA
#define CMD_PROM_C6 0xAC
#define CMD_PROM_C7 0xAE

#define PRESSURE_OSR_256 0x40
#define PRESSURE_OSR_512 0x42
#define PRESSURE_OSR_1024 0x44
#define PRESSURE_OSR_2048 0x46
#define PRESSURE_OSR_4096 0x48

#define TEMP_OSR_256 0x50
#define TEMP_OSR_512 0x52
#define TEMP_OSR_1024 0x54
#define TEMP_OSR_2048 0x56
#define TEMP_OSR_4096 0x58

/**
 * @brief The oversampling rate
 * @warn an higher value means a longer conversion
 */
typedef enum OSR {
	OSR_256,
	OSR_512,
	OSR_1024,
	OSR_2048,
	OSR_4096
}OSR;

typedef struct
{
	uint16_t C[8];
	uint32_t DigitalPressure_D1;
	uint32_t DigitalTemperature_D2;
	int32_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P;

	int OFF2;
	int T2;
	int SENS2;



}MS5611_t;

MS5611_t MS5611;


void MS5611_Reset(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611);
void MS5611_ReadProm(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct);

void MS5611_RequestTemperature(I2C_HandleTypeDef *I2Cx, OSR osr);
void MS5611_RequestPressure(I2C_HandleTypeDef *I2Cx, OSR osr);

void MS5611_ReadTemperature(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct);
void MS5611_ReadPressure(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct);

void MS5611_CalculateTemperature(MS5611_t *DataStruct);
void MS5611_CalculatePressure(MS5611_t *DataStruct);

#endif /* MS5611_H */
