# About this Library
The following guide is written in order to assist the use of GY-86 sensor module which combines MPU6050, HMC5883L, and MS5611 on a single PCB board. The library is intended to be used on STM32 microcontroller using HAL drivers utilizing I2C communication protocol. The overall structure of the project is inspired by leech001's library for the MPU6050. Here is the link to the repository, I hope you also check it out. </br>
https://github.com/leech001/MPU6050

## Table of Contents
1. MPU6050
2. HMC5883L (Understanding Auxiliary I2C)
3. MS5611

## 1. MPU6050
MPU6050 Sensor is fairly simple to use. All you have to use is the following functions created in this library. </br>

'''
MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG)
'''
This function is to set the full scale of the sensor. The range of output value is always consist of 16bit, however changing the full scale will change what the value means in real physical units.

## 2. HMC5883L
The HMC5883L sensor is not directly connected to SDA and SCL lines which is connected to the external pins. HMC5883L sensor is connected through auxiliary lines connected to MPU6050, which is not physically accessible through external pins on the board. In order to properly set up the HMC5883L, the MPU6050 must be in bypass mode so that the micro controller can simply bypass MPU6050 and directly access to the HMC5883L. Only then the microcontroller can write registers to HMC5883L sensor. Once initialization of HMC5883L is complete, the MPU6050 should be brought back to the master mode and it should read the data from HMC5883L. The data sent by the HMC5883L will be read by the MPU6050, then it will be saved to the MPU6050's internal register address. When the user requires to read the data, the user can retrieve 9 axis values through reading the registers from MPU6050. </br>
</br>
Summary of HMC5883L </br>
1. Set MPU6050 to bypass mode (it is set to master mode by default, therefore needs to be changed) </br>
2. Directly access HMC5883L for initial setup </br>
3. Set MPU6050 to back to master mode </br>
4. Let MPU6050 to read HMC5883L through auxiliary I2C connection </br>
5. Read 9 axis values from MPU6050 </br>


## 3. MS5611
On the other hand, MS5611 sensor is directly connected to SDA and SCL lines so it could be accessed from the master device without any issue.

