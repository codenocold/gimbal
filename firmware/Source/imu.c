#include "imu.h"
#include "mpu6050.h"
#include <math.h>

// 内部函数：3x3 矩阵求逆 (伴随矩阵法)
static int Matrix3x3_Inverse(float m[3][3], float inv[3][3])
{
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabsf(det) < 1e-6f)
        return 0; // 不可逆
    float invDet = 1.0f / det;

    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invDet;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invDet;
    inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invDet;
    return 1;
}

void EKF_Init(EKF_t *ekf, float dt, float q_noise, float r_noise)
{
    ekf->dt   = dt;
    ekf->q[0] = 1.0f;
    ekf->q[1] = 0.0f;
    ekf->q[2] = 0.0f;
    ekf->q[3] = 0.0f;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = (i == j) ? 0.01f : 0.0f;
            ekf->Q[i][j] = (i == j) ? q_noise : 0.0f;
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->R[i][j] = (i == j) ? r_noise : 0.0f;
        }
    }
}

void EKF_Update(EKF_t *ekf, float gx, float gy, float gz, float ax, float ay, float az)
{
    float q0 = ekf->q[0], q1 = ekf->q[1], q2 = ekf->q[2], q3 = ekf->q[3];
    float dt = ekf->dt;

    // --- 1. 预测 (Prediction) ---
    // 四元数更新 (Runge-Kutta 1st order)
    float dq0 = 0.5f * dt * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * dt * (q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * dt * (q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * dt * (q0 * gz + q1 * gy - q2 * gx);
    ekf->q[0] += dq0;
    ekf->q[1] += dq1;
    ekf->q[2] += dq2;
    ekf->q[3] += dq3;

    // 状态转移雅可比 F
    float F[4][4] = {
        {1.0f,           -0.5f * dt * gx, -0.5f * dt * gy, -0.5f * dt * gz},
        {0.5f * dt * gx, 1.0f,            0.5f * dt * gz,  -0.5f * dt * gy},
        {0.5f * dt * gy, -0.5f * dt * gz, 1.0f,            0.5f * dt * gx },
        {0.5f * dt * gz, 0.5f * dt * gy,  -0.5f * dt * gx, 1.0f           }
    };

    // P = F*P*F' + Q
    float Ft[4][4], FP[4][4], FPFt[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Ft[i][j] = F[j][i];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            FP[i][j] = 0;
            for (int k = 0; k < 4; k++)
                FP[i][j] += F[i][k] * ekf->P[k][j];
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            FPFt[i][j] = 0;
            for (int k = 0; k < 4; k++)
                FPFt[i][j] += FP[i][k] * Ft[k][j];
            ekf->P[i][j] = FPFt[i][j] + ekf->Q[i][j];
        }
    }

    // --- 2. 测量更新 (Update) ---
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f)
        return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    q0 = ekf->q[0];
    q1 = ekf->q[1];
    q2 = ekf->q[2];
    q3 = ekf->q[3];

    // 观测模型预测 h(x)
    float h[3] = {2.0f * (q1 * q3 - q0 * q2), 2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3};

    // 观测雅可比 H (3x4)
    float H[3][4] = {
        {-2.0f * q2, 2.0f * q3,  -2.0f * q0, 2.0f * q1},
        {2.0f * q1,  2.0f * q0,  2.0f * q3,  2.0f * q2},
        {2.0f * q0,  -2.0f * q1, -2.0f * q2, 2.0f * q3}
    };

    // S = H*P*H' + R
    float HP[3][4], S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            HP[i][j] = 0;
            for (int k = 0; k < 4; k++)
                HP[i][j] += H[i][k] * ekf->P[k][j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = ekf->R[i][j];
            for (int k = 0; k < 4; k++)
                S[i][j] += HP[i][k] * H[j][k]; // H[j][k] is H'
        }
    }

    // K = P*H' * inv(S)
    float invS[3][3];
    if (!Matrix3x3_Inverse(S, invS))
        return;

    float K[4][3];
    float PHt[4][3];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            PHt[i][j] = 0;
            for (int k = 0; k < 4; k++)
                PHt[i][j] += ekf->P[i][k] * H[j][k];
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0;
            for (int k = 0; k < 3; k++)
                K[i][j] += PHt[i][k] * invS[k][j];
        }
    }

    // 更新状态估计 q = q + K*(z - h)
    float dz[3] = {ax - h[0], ay - h[1], az - h[2]};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++)
            ekf->q[i] += K[i][j] * dz[j];
    }

    // 归一化四元数
    norm = sqrtf(ekf->q[0] * ekf->q[0] + ekf->q[1] * ekf->q[1] + ekf->q[2] * ekf->q[2] + ekf->q[3] * ekf->q[3]);
    ekf->q[0] /= norm;
    ekf->q[1] /= norm;
    ekf->q[2] /= norm;
    ekf->q[3] /= norm;

    // 更新 P = (I - K*H)*P
    float KH[4][4], I_KH[4][4], newP[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            KH[i][j] = 0;
            for (int k = 0; k < 3; k++)
                KH[i][j] += K[i][k] * H[k][j];
            I_KH[i][j] = ((i == j) ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            newP[i][j] = 0;
            for (int k = 0; k < 4; k++)
                newP[i][j] += I_KH[i][k] * ekf->P[k][j];
        }
    }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf->P[i][j] = newP[i][j];
}

void EKF_GetEulerAngles(EKF_t *ekf, float *roll, float *pitch, float *yaw)
{
    float q0 = ekf->q[0], q1 = ekf->q[1], q2 = ekf->q[2], q3 = ekf->q[3];
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}