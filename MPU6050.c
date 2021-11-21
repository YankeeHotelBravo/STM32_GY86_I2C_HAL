#include <math.h>
#include "mpu6050.h"

uint8_t MPU6050_rx;
uint8_t MPU6050_rx_buf[20];
uint8_t MPU6050_tx;
float MPU6050_Gyro_LSB = 32.8;
float MPU6050_Acc_LSB = 4096.0;

const float MPU6050_dt = 0.001;
const float MPU6050_alpha = 0.9996;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG)
{

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
		HAL_Delay(10);

		MPU6050_tx = 0x00; // Set No Sampling
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = DLPF_CFG; // Digital Low Pass Filter Setting
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = Gyro_FS << 3;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = Acc_FS << 3;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = 1; //Enable Data Ready Interrupt
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		//        MPU6050_tx = 0x00; //
		//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6A, 1, &MPU6050_tx, 1, 100);
		//
		//        MPU6050_tx = 0x02; //
		//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x37, 1, &MPU6050_tx, 1, 100);

		return 0;
	}

	return 1;
}

void MPU6050_Master(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 0x00; //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x37, 1, &MPU6050_tx, 1, 100);
	
	HAL_Delay(10);

	MPU6050_tx = 0x20; //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6A, 1, &MPU6050_tx, 1, 100);
}

void MPU6050_to_HMC5883L(I2C_HandleTypeDef *I2Cx)
{

	MPU6050_tx = 0b00011000;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x63, 1, &MPU6050_tx, 1, 100); //Value to Write to Config A
	HAL_Delay(10);
	MPU6050_tx = HMC5883L_ADDRESS;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x25, 1, &MPU6050_tx, 1, 100); //Slave Address (Write Mode)
	HAL_Delay(10);
	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x26, 1, &MPU6050_tx, 1, 100); //Slave Register (Config A)
	HAL_Delay(10);
	MPU6050_tx = 1 | 0x80; //Write 1 Byte
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100); //Enable + Length 1
	HAL_Delay(10);
	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100); //Disable Transfer
	HAL_Delay(10);


	HAL_Delay(10);

	MPU6050_tx = 0b00000000; //Fill Slave0 DO
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x63, 1, &MPU6050_tx, 1, 100); //Value to write to Config B
	HAL_Delay(10);
	MPU6050_tx = HMC5883L_ADDRESS; //Access HMC5883L in write mode
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x25, 1, &MPU6050_tx, 1, 100); //Slave Address (Write Mode)
	HAL_Delay(10);
	MPU6050_tx = 0x01; //Access to Config B of HMC5883L
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x26, 1, &MPU6050_tx, 1, 100); //Slave Register (Config B)
	HAL_Delay(10);
	MPU6050_tx = 1 | 0x80; //Write 1 Byte
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100); //Enable + Length 1
	HAL_Delay(10);
	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100); //Disable Transfer
	HAL_Delay(10);

	HAL_Delay(10);

	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x63, 1, &MPU6050_tx, 1, 100); //Value to write to Config Mode
	HAL_Delay(10);
	MPU6050_tx = HMC5883L_ADDRESS;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x25, 1, &MPU6050_tx, 1, 100); //Slave Address (Write Mode)
	HAL_Delay(10);
	MPU6050_tx = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x26, 1, &MPU6050_tx, 1, 100); //Slave Register (Config Mode)
	HAL_Delay(10);
	MPU6050_tx = 1 | 0x80; //Write 1 Byte
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100); //Enable + Length 1
	HAL_Delay(10);
	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x63, 1, &MPU6050_tx, 1, 100); //Disable Transfer

	HAL_Delay(10);
}

void MPU6050_Slave_Read(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = HMC5883L_ADDRESS | 0x10000000;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x25, 1, &MPU6050_tx, 1, 100); //Slave Address (Read Mode)
	HAL_Delay(10);
	MPU6050_tx = 0x03;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x26, 1, &MPU6050_tx, 1, 100); //Slave REG for reading to take place
	HAL_Delay(10);
	MPU6050_tx = 6 | 0x80; //Number of data bytes
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100); //Enable + Length 6
	HAL_Delay(10);
}

uint8_t MPU6050_DataReady(I2C_HandleTypeDef *I2Cx)
{
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, INT_STATUS_REG, 1, &MPU6050_rx, 1, 100);
	return MPU6050_rx;
}

void MPU6050_Read_Acc(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 6, 100);

	DataStruct->Accel_X_RAW = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Accel_Y_RAW = (int16_t) (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
	DataStruct->Accel_Z_RAW = (int16_t) (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, MPU6050_rx_buf, 6, 100);

	DataStruct->Gyro_X_RAW = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Gyro_Y_RAW = (int16_t) (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
	DataStruct->Gyro_Z_RAW = (int16_t) (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);

	DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 20, 10);

	DataStruct->Accel_X_RAW = -(MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Accel_Y_RAW = (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
	DataStruct->Accel_Z_RAW = (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);
	// Didn't Save Temp Value
	DataStruct->Gyro_X_RAW = (MPU6050_rx_buf[8] << 8 | MPU6050_rx_buf[9]);
	DataStruct->Gyro_Y_RAW = -(MPU6050_rx_buf[10] << 8 | MPU6050_rx_buf[11]);
	DataStruct->Gyro_Z_RAW = -(MPU6050_rx_buf[12] << 8 | MPU6050_rx_buf[13]);

	DataStruct->Mag_X_RAW = (MPU6050_rx_buf[14] << 8 | MPU6050_rx_buf[15]);
	DataStruct->Mag_Z_RAW = -(MPU6050_rx_buf[16] << 8 | MPU6050_rx_buf[17]);
	DataStruct->Mag_Y_RAW = -(MPU6050_rx_buf[18] << 8 | MPU6050_rx_buf[19]);


	DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
	DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
	DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

//	DataStruct->Mag_X_RAW -= DataStruct->Mag_X_Offset;
//	DataStruct->Mag_Y_RAW -= DataStruct->Mag_Y_Offset;
//	DataStruct->Mag_Z_RAW -= DataStruct->Mag_Z_Offset;

	DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB* D2R;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB* D2R;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB* D2R;
	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	int16_t temp;

	// Read 2 BYTES of data starting from TEMP_OUT_H_REG register

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, MPU6050_rx_buf, 2, 100);

	temp = (int16_t) (MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}

void MPU6050_Read_All_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{

	HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 20);
}

void MPU6050_Parsing(MPU6050_t *DataStruct)
{
	DataStruct->Accel_X_RAW = -(MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Accel_Y_RAW = (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
	DataStruct->Accel_Z_RAW = (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);
	// Didn't Save Temp Value
	DataStruct->Gyro_X_RAW = (MPU6050_rx_buf[8] << 8 | MPU6050_rx_buf[9]);
	DataStruct->Gyro_Y_RAW = -(MPU6050_rx_buf[10] << 8 | MPU6050_rx_buf[11]);
	DataStruct->Gyro_Z_RAW = -(MPU6050_rx_buf[12] << 8 | MPU6050_rx_buf[13]);

	DataStruct->Mag_X_RAW = (MPU6050_rx_buf[14] << 8 | MPU6050_rx_buf[15]);
	DataStruct->Mag_Z_RAW = -(MPU6050_rx_buf[16] << 8 | MPU6050_rx_buf[17]);
	DataStruct->Mag_Y_RAW = -(MPU6050_rx_buf[18] << 8 | MPU6050_rx_buf[19]);


	DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
	DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
	DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

	DataStruct->Mag_X_RAW -= DataStruct->Mag_X_Offset;
	DataStruct->Mag_Y_RAW -= DataStruct->Mag_Y_Offset;
	DataStruct->Mag_Z_RAW -= DataStruct->Mag_Z_Offset;

	DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB* D2R;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB* D2R;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB* D2R;
	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}
