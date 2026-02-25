#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

typedef struct
{
    float q0, q1, q2, q3;            // 四元数
    float gx_bias, gy_bias, gz_bias; // 陀螺零偏
} EKF_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} Attitude_t;

void IMU_Init(EKF_t *ekf);
void IMU_Update(EKF_t *ekf,
                float  gx,
                float  gy,
                float  gz, // 陀螺 deg/s
                float  ax,
                float  ay,
                float  az, // 加速度
                float  dt); // 时间间隔 s

void IMU_GetAttitude(EKF_t *ekf, Attitude_t *att);

#endif /* __IMU_H__ */