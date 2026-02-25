#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"

int32_t MPU6050_init(void);
int32_t MPU6050_read_raw_data(int16_t *acce_x, int16_t *acce_y, int16_t *acce_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z, int16_t *temper);

#endif /* __MPU6050_H__ */