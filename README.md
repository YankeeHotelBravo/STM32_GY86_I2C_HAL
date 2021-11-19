# GY-86
For the use of GY-86 sensor which combines MPU6050, HMC5883L, and MS5611 on a single PCB module. The library is intended to be used on STM32 microcontroller using HAL drivers on I2C communication protocol. The overall structure of the project is inspired by leech001's library for the MPU6050. Here is the link to the repository, I hope you also check it out.

https://github.com/leech001/MPU6050

The HMC5883L sensor is not directly connected to SDA and SCL lines. HMC5883L sensor is connected to MPU6050 sensor, therefore it should be controlled and read through MPU6050. There are some initialization functions that helps setting MPU6050 as a master device and allow it to access to HMC5883L.

On the other hand, MS5611 sensor is directly connected to SDA and SCL lines so it could be accessed from the master device without any issue.

Completed:<br/>
Reading 6 Axis info from MPU6050 <br/>
Accessing HMC5883L through MPU6050 <br/>
Reading data from MS5611 <br/>

WIP: <br/>
HMC5883L Calibration
MS5611 Stabilization
