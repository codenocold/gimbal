#include "imu.h"
#include "mpu6050.h"
#include <math.h>

#define RAD 0.0174532925f
#define DEG 57.2957795f

// 四元数归一化
static void qNorm(float *q0, float *q1, float *q2, float *q3)
{
    float norm = sqrtf((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    if (norm > 1e-6f) {
        *q0 /= norm;
        *q1 /= norm;
        *q2 /= norm;
        *q3 /= norm;
    }
}

// 初始化
void IMU_Init(EKF_t *ekf)
{
    ekf->q0      = 1.0f;
    ekf->q1      = 0.0f;
    ekf->q2      = 0.0f;
    ekf->q3      = 0.0f;
    ekf->gx_bias = 0.0f;
    ekf->gy_bias = 0.0f;
    ekf->gz_bias = 0.0f;
}

// 无磁力计 IMU 更新（核心）
void IMU_Update(EKF_t *ekf, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float q0 = ekf->q0;
    float q1 = ekf->q1;
    float q2 = ekf->q2;
    float q3 = ekf->q3;

    float gbx = ekf->gx_bias;
    float gby = ekf->gy_bias;
    float gbz = ekf->gz_bias;

    // 陀螺去零偏，转 rad/s
    float gx_r = (gx - gbx) * RAD;
    float gy_r = (gy - gby) * RAD;
    float gz_r = (gz - gbz) * RAD;

    // ==========================
    // 1. 预测：四元数更新
    // ==========================
    float dq0 = 0.5f * (-q1 * gx_r - q2 * gy_r - q3 * gz_r);
    float dq1 = 0.5f * (q0 * gx_r - q3 * gy_r + q2 * gz_r);
    float dq2 = 0.5f * (q3 * gx_r + q0 * gy_r - q1 * gz_r);
    float dq3 = 0.5f * (-q2 * gx_r + q1 * gy_r + q0 * gz_r);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    qNorm(&q0, &q1, &q2, &q3);

    // ==========================
    // 2. 观测：加速度修正 pitch/roll
    // ==========================
    float norm_a = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_a < 0.1f) {
        ekf->q0 = q0;
        ekf->q1 = q1;
        ekf->q2 = q2;
        ekf->q3 = q3;
        return;
    }

    // 归一化加速度
    ax /= norm_a;
    ay /= norm_a;
    az /= norm_a;

    // 机体坐标系下的重力方向
    float gx_b = 2 * (q1 * q3 - q0 * q2);
    float gy_b = 2 * (q0 * q1 + q2 * q3);

    // 误差
    float ex = ax - gx_b;
    float ey = ay - gy_b;

    // 比例增益（调这个！）
    const float Kp = 0.2f;    // 姿态修正
    const float Ki = 0.0005f; // 零偏修正

    // 修正四元数
    q1 += Kp * ex * dt;
    q2 += Kp * ey * dt;

    // 估计并修正陀螺零偏
    gbx += Ki * ex * dt;
    gby += Ki * ey * dt;

    qNorm(&q0, &q1, &q2, &q3);

    // 回写
    ekf->q0      = q0;
    ekf->q1      = q1;
    ekf->q2      = q2;
    ekf->q3      = q3;
    ekf->gx_bias = gbx;
    ekf->gy_bias = gby;
    ekf->gz_bias = gbz;
}

// 四元数转欧拉角
void IMU_GetAttitude(EKF_t *ekf, Attitude_t *att)
{
    float q0 = ekf->q0;
    float q1 = ekf->q1;
    float q2 = ekf->q2;
    float q3 = ekf->q3;

    att->roll  = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * DEG;
    att->pitch = asinf(2 * (q0 * q2 - q3 * q1)) * DEG;
    att->yaw   = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * DEG;

    if (att->yaw < 0)
        att->yaw += 360.0f;
}