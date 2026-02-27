#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

typedef struct
{
    float q[4];    // 状态向量：四元数 [q0, q1, q2, q3]
    float P[4][4]; // 状态协方差矩阵
    float Q[4][4]; // 过程噪声协方差
    float R[3][3]; // 测量噪声协方差
    float dt;      // 采样周期 (单位: s)
    float acc_ref[3];          // 低通重力参考向量 (机体系, 单位向量)
    uint8_t acc_ref_valid;     // 参考向量是否已初始化
    uint8_t acc_reject_cooldown; // 加速度观测拒绝冷却计数
} EKF_t;

// 初始化 EKF 结构体
void EKF_Init(EKF_t *ekf, float dt, float q_noise, float r_noise);

// EKF 核心更新函数：传入陀螺仪(rad/s)和加速度计(m/s^2 或 g)
void EKF_Update(EKF_t *ekf, float gx, float gy, float gz, float ax, float ay, float az);

// 获取欧拉角 (单位: 度)
void EKF_GetEulerAngles(EKF_t *ekf, float *roll, float *pitch, float *yaw);

#endif /* __IMU_H__ */