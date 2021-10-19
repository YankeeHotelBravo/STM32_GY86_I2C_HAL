/*
 * barometer.c
 *
 *  Created on: 5 oct. 2018
 *      Author: alex
 */

#include "MS5611.h"
#include <math.h>

uint8_t MS5611_rx_buf[12];
uint8_t MS5611_tx;
uint8_t MS5611_rx;


void MS5611_Reset(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct)
{
	MS5611_tx = CMD_RESET;
	HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDR, &MS5611_tx, 1, 100);

	//For Temperature > 20 Celsius
	DataStruct->T2 = 0;
	DataStruct->OFF2 = 0;
	DataStruct->SENS2 = 0;
}

void MS5611_ReadProm(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct)
{
	for(int i=0 ; i<8 ; i++)
	{
		MS5611_tx = CMD_PROM_C0;
		HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDR, &MS5611_tx, 1, 100);
		HAL_I2C_Master_Receive(I2Cx, MS5611_ADDR, MS5611_rx_buf, 2, 100);
		DataStruct->C[i] = MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1];
		MS5611_tx += 2;
	}
}


void MS5611_RequestTemperature(I2C_HandleTypeDef *I2Cx, OSR osr)
{
	MS5611_tx = TEMP_OSR_256 + (2 * osr);
	HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDR, &MS5611_tx, 1, 100);
}

void MS5611_RequestPressure(I2C_HandleTypeDef *I2Cx, OSR osr)
{
	MS5611_tx = PRESSURE_OSR_256 + (2 * osr);
	HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDR, &MS5611_tx, 1, 100);
}

void MS5611_ReadTemperature(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct)
{
	//Read ADC
	HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, 0x00, 1, MS5611_rx_buf, 3, 100);

	DataStruct->DigitalTemperature_D2 = MS5611_rx_buf[0] << 16 | MS5611_rx_buf[1] << 8 | MS5611_rx_buf[0];
}

void MS5611_ReadPressure(I2C_HandleTypeDef *I2Cx, MS5611_t *DataStruct)
{
	//Read ADC
	HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, 0x00, 1, MS5611_rx_buf, 3, 100);

	DataStruct->DigitalPressure_D1 = MS5611_rx_buf[0] << 16 | MS5611_rx_buf[1] << 8 | MS5611_rx_buf[0];
}

void MS5611_CalculateTemperature(MS5611_t *DataStruct)
{
	DataStruct->dT = DataStruct->C[5];
	DataStruct->dT <<= 8; //Calculated up to C5 * 2^8
	DataStruct->dT *= -1; //Apply negative sign
	DataStruct->dT += DataStruct->DigitalTemperature_D2; // = D2 - C5 * 2^8

	DataStruct->TEMP = DataStruct->dT * DataStruct->C[6];
	DataStruct->TEMP >>= 23; // Calculated up to dT * C6 / 2^23
	DataStruct->TEMP += 2000;
}

void MS5611_CalculatePressure(MS5611_t *DataStruct)
{
	DataStruct->OFF = DataStruct->C[2];
	DataStruct->OFF <<= 16; //Calculated up to C2 * 2^16
	DataStruct->OFF += (DataStruct->C[4] * DataStruct->dT) >> 7;


	DataStruct->SENS = DataStruct->C[1];
	DataStruct->SENS <<= 15; // Calculated up to C1 * 2^15
	DataStruct->SENS += (DataStruct->C[3] * DataStruct->dT) >>8;

	DataStruct->P = ((DataStruct->DigitalPressure_D1 * DataStruct->SENS) / pow(2, 21) - DataStruct->OFF) / pow(2, 15);
}
