# GY-86
For the use of GY-86 sensor which combines MPU6050, HMC5883L, and MS5611 on a single PCB module. The library is intended to be used on STM32 microcontroller using HAL drivers on I2C communication protocol. The overall structure of the project is inspired by leech001's library for the MPU6050. Here is the link to the repository, I hope you also check it out.

https://github.com/leech001/MPU6050

The HMC5883L sensor is not directly connected to SDA and SCL lines. HMC5883L sensor is connected through auxiliary lines of MPU6050. For setting up the HMC5883L, MPU6050 must be in bypass mode so that the controller can directly access to the HMC5883L and write it's registers. Once initialization of HMC5883L is complete, the MPU6050 is brought back to master mode and it reads the magnetometer value by itself. Then the user can retrieve 9 axis values through reading the registers from MPU6050 only.

Summary of HMC5883L
1. Set MPU6050 to bypass mode
2. Directly access HMC5883L for initial setup
3. Set MPU6050 to master mode
4. Let MPU6050 to read HMC5883L through auxiliary I2C connection
5. Read 9 axis values from MPU6050

On the other hand, MS5611 sensor is directly connected to SDA and SCL lines so it could be accessed from the master device without any issue.

Completed:<br/>
Reading 6 Axis info from MPU6050 <br/>
Accessing HMC5883L through MPU6050 <br/>
Reading data from MS5611 <br/>
HMC5883L Calibration <br/>
MS5611 Stabilization <br/>

WIP: <br/>

