#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "i2c.h"

#define MPU6050_ADDR         0x68

#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    float temperature;
} MPU6050_Data;

typedef struct {
    float accel_x;  // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;   // °/s
    float gyro_y;
    float gyro_z;
    float temperature;  // °C
} MPU6050_ConvertedData;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Check(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_RawData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz);
void MPU6050_Convert_Data(MPU6050_Data *raw, MPU6050_ConvertedData *converted);

#endif