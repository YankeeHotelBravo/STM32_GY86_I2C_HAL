#ifndef MPU6050_H
#define MPU6050_H

//#include <stdint.h>
#include "main.h"

#define D2R 0.01745329252 //Degree To Radian Constant

#define MPU6050_ADDR 0xD0 //Already Left Shifted
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define SMPLRT_DIV_REG 0x19
#define INT_PIN_CFG 0x37
#define INT_ENABLE_REG 0x38
#define INT_STATUS_REG 0x3A
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define USER_CTRL_REG 0x6A
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

/////////////////////////////////////
#define HMC5883L_ADDRESS              0x1E // Not Left Shifted
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

// MPU6050 structure
typedef struct _MPU6050_t
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    int16_t Gyro_X_Offset;
    int16_t Gyro_Y_Offset;
    int16_t Gyro_Z_Offset;

    float Gx;
    float Gy;
    float Gz;

    float Temperature;

    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;

    int16_t Mag_X_Min;
	int16_t Mag_Y_Min;
	int16_t Mag_Z_Min;

    int16_t Mag_X_Max;
	int16_t Mag_Y_Max;
	int16_t Mag_Z_Max;

    int16_t Mag_X_Offset;
	int16_t Mag_Y_Offset;
	int16_t Mag_Z_Offset;

    float Mx;
    float My;
    float Mz;

}MPU6050_t;

MPU6050_t MPU6050;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t ACC_FS, uint8_t DLPF_CFG);

void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx);
void MPU6050_Master(I2C_HandleTypeDef *I2Cx);
void HMC5883L_Setup(I2C_HandleTypeDef *I2Cx);
void MPU6050_Slave_Read(I2C_HandleTypeDef *I2Cx);

uint8_t MPU6050_DataReady(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_All_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Parsing(MPU6050_t *DataStruct);
void MPU6050_Parsing_NoOffest(MPU6050_t *DataStruct);

#endif /* MPU6050_H */
