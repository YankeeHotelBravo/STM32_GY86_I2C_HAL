#include <math.h>
#include "mpu6050.h"

uint8_t MPU6050_rx;
uint8_t MPU6050_rx_buf[13];
uint8_t MPU6050_tx;
float MPU6050_Gyro_LSB;
float MPU6050_Acc_LSB;

const float MPU6050_dt = 0.001;
const float MPU6050_alpha = 0.996;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG) {

    //Save LSB/Unit for both gyro and acc in order to use them later
    switch(Gyro_FS)
    {
    case 0: //250dps
    	MPU6050_Gyro_LSB = 131.0;
    	break;

    case 1: //500dps
    	MPU6050_Gyro_LSB = 65.5;
    	break;

    case 2: //1000dps
    	MPU6050_Gyro_LSB = 32.8;
    	break;

    case 3: //2000dps
    	MPU6050_Gyro_LSB = 16.4;
    	break;

    default:
    	break;
    }

    switch(Acc_FS)
    {
    case 0: //2g
    	MPU6050_Acc_LSB = 16384.0;
    	break;

    case 1: //4g
    	MPU6050_Acc_LSB = 8192.0;
    	break;

    case 2: //8g
    	MPU6050_Acc_LSB = 4096.0;
    	break;

    case 3: //16g
    	MPU6050_Acc_LSB = 2048.0;
    	break;

    default:
    	break;
    }

    // Read Who am I
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &MPU6050_rx, 1, 100);
    MPU6050_tx = 0; //Will return this value if code ends here

    // 0x68 will be returned if sensor accessed correctly
    if (MPU6050_rx == 0x68)
    {
        MPU6050_tx = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &MPU6050_tx, 1, 100);

        MPU6050_tx = 0x00; // Set No Sampling
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &MPU6050_tx, 1, 100);

        MPU6050_tx = DLPF_CFG; // Digital Low Pass Filter Setting
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &MPU6050_tx, 1, 100);

        MPU6050_tx = Gyro_FS << 3;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &MPU6050_tx, 1, 100);

        MPU6050_tx = Acc_FS << 3;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &MPU6050_tx, 1, 100);

        MPU6050_tx = 1; //Enable Data Ready Interrupt
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, 1, &MPU6050_tx, 1, 100);

        MPU6050_tx = 1; //Will return this value if settings are completed
    }

    return MPU6050_tx;
}

uint8_t MPU6050_DataReady(I2C_HandleTypeDef *I2Cx)
{
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, INT_STATUS_REG, 1, &MPU6050_rx, 1, 100);
	return MPU6050_rx;
}

void MPU6050_Read_Acc(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 6, 100);

    DataStruct->Accel_X_RAW = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
    DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, MPU6050_rx_buf, 6, 100);

    DataStruct->Gyro_X_RAW = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 14, 100);

    DataStruct->Accel_X_RAW = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);
    // Didn't Save Temp Value
    DataStruct->Gyro_X_RAW = (int16_t) (MPU6050_rx_buf[8] << 8 | MPU6050_rx_buf[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (MPU6050_rx_buf[10] << 8 | MPU6050_rx_buf[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (MPU6050_rx_buf[12] << 8 | MPU6050_rx_buf[13]);

    DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
    DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
    DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;


    DataStruct->Gx = (DataStruct->Gyro_X_RAW) / MPU6050_Gyro_LSB;
    DataStruct->Gy = -(DataStruct->Gyro_Y_RAW) / MPU6050_Gyro_LSB;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW) / MPU6050_Gyro_LSB;
    DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
    DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;


    DataStruct->Gyro_Pitch += DataStruct->Gx * MPU6050_dt;
    DataStruct->Gyro_Roll += DataStruct->Gy * MPU6050_dt;
    DataStruct->Gyro_Pitch -= DataStruct->Roll * sin(DataStruct->Gz * MPU6050_dt * D2R);
	DataStruct->Gyro_Roll += DataStruct->Pitch * sin(DataStruct->Gz * MPU6050_dt * D2R);


	DataStruct->acc_total_vector=sqrtf(DataStruct->Accel_X_RAW*DataStruct->Accel_X_RAW + DataStruct->Accel_Y_RAW*DataStruct->Accel_Y_RAW + DataStruct->Accel_Z_RAW*DataStruct->Accel_Z_RAW);
    if( abs(DataStruct->Accel_X_RAW) < DataStruct->acc_total_vector)
    {
    	DataStruct->Acc_Pitch=asinf((float)DataStruct->Accel_X_RAW/DataStruct->acc_total_vector)*(57.29577951);
    }
    if( abs(DataStruct->Accel_Y_RAW) < DataStruct->acc_total_vector)
    {
    	DataStruct->Acc_Roll=asinf((float)DataStruct->Accel_Y_RAW/DataStruct->acc_total_vector)*(57.29577951);
    }


  	DataStruct->Pitch =MPU6050_alpha*DataStruct->Gyro_Pitch + (1-MPU6050_alpha)*(DataStruct->Acc_Pitch);
  	DataStruct->Roll =MPU6050_alpha*DataStruct->Gyro_Roll + (1-MPU6050_alpha)*(DataStruct->Acc_Roll);
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, MPU6050_rx_buf, 2, 100);

    temp = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}
