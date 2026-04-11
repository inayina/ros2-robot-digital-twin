#include "mpu6050.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data = 0;
    HAL_StatusTypeDef status;

    HAL_Delay(50);

    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, MPU6050_REG_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }

    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, MPU6050_REG_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }

    data = 0x07;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, MPU6050_REG_SMPLRT_DIV,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }

    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, MPU6050_REG_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }

    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, MPU6050_REG_GYRO_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }

    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, MPU6050_REG_ACCEL_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }

    return MPU6050_Check(hi2c);
}

uint8_t MPU6050_Check(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, MPU6050_REG_WHO_AM_I,
                      I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);
    if (status != HAL_OK) {
        return 0;
    }
    return (who_am_i == 0x68) ? 1 : 0;
}

void MPU6050_Read_RawData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data)
{
    uint8_t buffer[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, MPU6050_REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, buffer, 14, 1000);

    data->ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->az = (int16_t)(buffer[4] << 8 | buffer[5]);
    data->temperature = (int16_t)(buffer[6] << 8 | buffer[7]) / 340.0f + 36.53f;
    data->gx = (int16_t)(buffer[8] << 8 | buffer[9]);
    data->gy = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gz = (int16_t)(buffer[12] << 8 | buffer[13]);
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, MPU6050_REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, buffer, 6, 1000);
    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, MPU6050_REG_GYRO_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, buffer, 6, 1000);
    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);
}

void MPU6050_Convert_Data(MPU6050_Data *raw, MPU6050_ConvertedData *converted)
{
    // 加速度转换：默认量程 ±2g, 灵敏度 16384 LSB/g
    // 转换为 m/s² (g = 9.80665 m/s²)
    const float ACCEL_SCALE = 16384.0f;
    const float GRAVITY = 9.80665f;
    converted->accel_x = (float)raw->ax / ACCEL_SCALE * GRAVITY;
    converted->accel_y = (float)raw->ay / ACCEL_SCALE * GRAVITY;
    converted->accel_z = (float)raw->az / ACCEL_SCALE * GRAVITY;

    // 陀螺仪转换：默认量程 ±250°/s, 灵敏度 131 LSB/°/s
    const float GYRO_SCALE = 131.0f;
    converted->gyro_x = (float)raw->gx / GYRO_SCALE;
    converted->gyro_y = (float)raw->gy / GYRO_SCALE;
    converted->gyro_z = (float)raw->gz / GYRO_SCALE;

    // 温度已经转换
    converted->temperature = raw->temperature;
}