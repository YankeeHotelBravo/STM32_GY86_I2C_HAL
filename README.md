# GY-86
For use of GY-86 sensor which combines MPU6050, HMC5883L, and MS5611 on one single PCB module. It is intended to be used on STM32 microcontroller using HAL drivers on I2C communication protocol.

The HMC5883L sensor is not directly connected to SDA and SCL lines. HMC5883L sensor is connected to MPU6050 sensor, therefore it should be controlled and read through MPU6050. There are some initialization functions that helps setting MPU6050 as a master device and allow it to access to HMC5883L.

On the other hand, MS5611 sensor is directly connected to SDA and SCL lines so it could be accessed from the master device without any issue.

Completed: \n
Reading 6 Axis info from MPU6050 \n
Accessing HMC5883L through MPU6050 \n
Reading data from MS5611 \n

WIP: \n
HMC5883L Calibration
