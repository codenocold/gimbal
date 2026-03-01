/*
 * EKF.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Emre Emir Fidan
 */

#include "EKF.h"
#include "math.h"

void EKF_init(ekf_t *ekf, float ref_mx, float ref_my, float ref_mz, float N_Q, float N_Q_bias, float N_P, float N_R, uint8_t use_mag)
{
    // Initialization:
    // Prediction error covariance matrix
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            if (i == j) {
                ekf->P[i][j] = N_P;
            } else {
                ekf->P[i][j] = 0.0f;
            }
        }
    }

    // Process noise covariance matrix
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            if (i == j) {
                ekf->Q[i][j] = (i < 4) ? N_Q : N_Q_bias;
            } else {
                ekf->Q[i][j] = 0.0f;
            }
        }
    }

    // Measurement noise covariance matrix
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            if (i == j) {
                ekf->R[i][j] = N_R;
            } else {
                ekf->R[i][j] = 0.0f;
            }
        }
    }

    ekf->x[0] = 1;
    ekf->x[1] = 0;
    ekf->x[2] = 0;
    ekf->x[3] = 0;
    ekf->x[4] = 0;
    ekf->x[5] = 0;
    ekf->x[6] = 0;

    // Normalize Reference Magnetic Vector
    float M      = sqrtf(ref_mx * ref_mx + ref_my * ref_my + ref_mz * ref_mz);
    ekf->ref_mx  = ref_mx / M;
    ekf->ref_my  = ref_my / M;
    ekf->ref_mz  = ref_mz / M;
    ekf->use_mag = (use_mag != 0u) ? 1u : 0u;
}

void EKF_update(ekf_t *ekf, float euler[3], float ax, float ay, float az, float p, float q, float r, float mx, float my, float mz, float dt)
{
    // Variable Definitions
    float   F[EKF_STATE_DIM][EKF_STATE_DIM]; // Jacobian matrix of F
    float   H[EKF_MEAS_DIM][EKF_STATE_DIM];  // Jacobian matrix of H
    float   FP[EKF_STATE_DIM][EKF_STATE_DIM];
    float   FPFt[EKF_STATE_DIM][EKF_STATE_DIM];
    float   HPp[EKF_MEAS_DIM][EKF_STATE_DIM];
    float   HPpHt[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float   PpHt[EKF_STATE_DIM][EKF_MEAS_DIM];
    float   S_inv[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float   Hxp[EKF_MEAS_DIM];
    float   z[EKF_MEAS_DIM];
    float   zmHxp[EKF_MEAS_DIM];
    float   KzmHxp[EKF_STATE_DIM];
    float   KH[EKF_STATE_DIM][EKF_STATE_DIM];
    float   KHPp[EKF_STATE_DIM][EKF_STATE_DIM];
    float   G;
    float   M;
    float   q0;
    float   q1;
    float   q2;
    float   q3;
    float   bgx;
    float   bgy;
    float   bgz;
    float   wx;
    float   wy;
    float   wz;
    float   q_norm;
    float   gyro_norm;
    float   bias_track;
    uint8_t is_static;
    uint8_t mat_error;

    // Normalization
    G = sqrtf(ax * ax + ay * ay + az * az);
    if (G > 0.0f) {
        ax = ax / G;
        ay = ay / G;
        az = az / G;
    }
    if (ekf->use_mag != 0u) {
        M  = sqrtf(mx * mx + my * my + mz * mz);
        mx = mx / M;
        my = my / M;
        mz = mz / M;
    } else {
        mx = 0.0f;
        my = 0.0f;
        mz = 0.0f;
    }

    q0  = ekf->x[0];
    q1  = ekf->x[1];
    q2  = ekf->x[2];
    q3  = ekf->x[3];
    bgx = ekf->x[4];
    bgy = ekf->x[5];
    bgz = ekf->x[6];

    wx        = p - bgx;
    wy        = q - bgy;
    wz        = r - bgz;
    gyro_norm = sqrtf(p * p + q * q + r * r);
    is_static = (ekf->use_mag == 0u) && (fabsf(G - 1.0f) < 0.15f) && (gyro_norm < 0.35f);

    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Calculate the Jacobian matrix of F
    F[0][1] = -wx * dt / 2;
    F[0][2] = -wy * dt / 2;
    F[0][3] = -wz * dt / 2;
    F[0][4] = q1 * dt / 2;
    F[0][5] = q2 * dt / 2;
    F[0][6] = q3 * dt / 2;

    F[1][0] = wx * dt / 2;
    F[1][2] = wz * dt / 2;
    F[1][3] = -wy * dt / 2;
    F[1][4] = -q0 * dt / 2;
    F[1][5] = q3 * dt / 2;
    F[1][6] = -q2 * dt / 2;

    F[2][0] = wy * dt / 2;
    F[2][1] = -wz * dt / 2;
    F[2][3] = wx * dt / 2;
    F[2][4] = -q3 * dt / 2;
    F[2][5] = -q0 * dt / 2;
    F[2][6] = q1 * dt / 2;

    F[3][0] = wz * dt / 2;
    F[3][1] = wy * dt / 2;
    F[3][2] = -wx * dt / 2;
    F[3][4] = q2 * dt / 2;
    F[3][5] = -q1 * dt / 2;
    F[3][6] = -q0 * dt / 2;

    // Prediction of x
    ekf->xp[0] = q0 + (-wx * q1 - wy * q2 - wz * q3) * dt / 2;
    ekf->xp[1] = q1 + (wx * q0 + wz * q2 - wy * q3) * dt / 2;
    ekf->xp[2] = q2 + (wy * q0 - wz * q1 + wx * q3) * dt / 2;
    ekf->xp[3] = q3 + (wz * q0 + wy * q1 - wx * q2) * dt / 2;
    ekf->xp[4] = bgx;
    ekf->xp[5] = bgy;
    ekf->xp[6] = bgz;

    q_norm = sqrtf(ekf->xp[0] * ekf->xp[0] + ekf->xp[1] * ekf->xp[1] + ekf->xp[2] * ekf->xp[2] + ekf->xp[3] * ekf->xp[3]);
    if (q_norm > 0.0f) {
        ekf->xp[0] /= q_norm;
        ekf->xp[1] /= q_norm;
        ekf->xp[2] /= q_norm;
        ekf->xp[3] /= q_norm;
    }

    // Prediction of P
    /* Pp = F*P*F' + Q; */
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            FP[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_DIM; k++) {
                FP[i][j] += F[i][k] * ekf->P[k][j];
            }
        }
    }

    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            FPFt[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_DIM; k++) {
                FPFt[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            ekf->Pp[i][j] = FPFt[i][j] + ekf->Q[i][j];
        }
    }

    // Calculate the Jacobian matrix of h
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            H[i][j] = 0.0f;
        }
    }

    q0 = ekf->xp[0];
    q1 = ekf->xp[1];
    q2 = ekf->xp[2];
    q3 = ekf->xp[3];

    H[0][0] = -q2;
    H[0][1] = q3;
    H[0][2] = -q0;
    H[0][3] = q1;

    H[1][0] = q1;
    H[1][1] = q0;
    H[1][2] = q3;
    H[1][3] = q2;

    H[2][0] = q0;
    H[2][1] = -q1;
    H[2][2] = -q2;
    H[2][3] = q3;

    if (ekf->use_mag != 0u) {
        H[3][0] = q0 * ekf->ref_mx + q3 * ekf->ref_my - q2 * ekf->ref_mz;
        H[3][1] = q1 * ekf->ref_mx + q2 * ekf->ref_my + q3 * ekf->ref_mz;
        H[3][2] = -q2 * ekf->ref_mx + q1 * ekf->ref_my - q0 * ekf->ref_mz;
        H[3][3] = -q3 * ekf->ref_mx + q0 * ekf->ref_my + q1 * ekf->ref_mz;

        H[4][0] = -q3 * ekf->ref_mx + q0 * ekf->ref_my + q1 * ekf->ref_mz;
        H[4][1] = q2 * ekf->ref_mx - q1 * ekf->ref_my + q0 * ekf->ref_mz;
        H[4][2] = q1 * ekf->ref_mx + q2 * ekf->ref_my + q3 * ekf->ref_mz;
        H[4][3] = -q0 * ekf->ref_mx - q3 * ekf->ref_my + q2 * ekf->ref_mz;

        H[5][0] = q2 * ekf->ref_mx - q1 * ekf->ref_my + q0 * ekf->ref_mz;
        H[5][1] = q3 * ekf->ref_mx - q0 * ekf->ref_my - q1 * ekf->ref_mz;
        H[5][2] = q0 * ekf->ref_mx + q3 * ekf->ref_my - q2 * ekf->ref_mz;
        H[5][3] = q1 * ekf->ref_mx + q2 * ekf->ref_my + q3 * ekf->ref_mz;
    }

    // S = (H*Pp*H' + R);
    // H*Pp
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            HPp[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_DIM; k++) {
                HPp[i][j] += H[i][k] * ekf->Pp[k][j];
            }
        }
    }

    // H*Pp*H'
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            HPpHt[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_DIM; k++) {
                HPpHt[i][j] += HPp[i][k] * H[j][k];
            }
        }
    }

    // H*Pp*H' + R
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            HPpHt[i][j] = HPpHt[i][j] + ekf->R[i][j]; // S
        }
    }

    // K = Pp*H'*(S.inverse);
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            PpHt[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_DIM; k++) {
                PpHt[i][j] += ekf->Pp[i][k] * H[j][k];
            }
        }
    }

    mat_error = inverse_matrix(HPpHt, S_inv);
    if (!mat_error) {
        return;
    }

    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            ekf->K[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_MEAS_DIM; k++) {
                ekf->K[i][j] += PpHt[i][k] * S_inv[k][j];
            }
        }
    }

    // x = xp + K*(z - H*xp);
    // H*xp
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        Hxp[i] = 0.0f;
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            Hxp[i] += ekf->xp[j] * H[i][j];
        }
    }

    z[0] = ax;
    z[1] = ay;
    z[2] = az;
    if (ekf->use_mag != 0u) {
        z[3] = mx;
        z[4] = my;
        z[5] = mz;
    } else {
        z[3] = 0.0f;
        z[4] = 0.0f;
        z[5] = 0.0f;
    }

    // (z - H*xp)
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        zmHxp[i] = (z[i] - Hxp[i]);
    }

    // K*(z - H*xp)
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        KzmHxp[i] = 0;
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            KzmHxp[i] += ekf->K[i][j] * zmHxp[j];
        }
    }

    // xp + K*(z - H*xp)
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        ekf->x[i] = ekf->xp[i] + KzmHxp[i];
    }

    q_norm = sqrtf(ekf->x[0] * ekf->x[0] + ekf->x[1] * ekf->x[1] + ekf->x[2] * ekf->x[2] + ekf->x[3] * ekf->x[3]);
    if (q_norm > 0.0f) {
        ekf->x[0] /= q_norm;
        ekf->x[1] /= q_norm;
        ekf->x[2] /= q_norm;
        ekf->x[3] /= q_norm;
    }

    if (is_static) {
        bias_track = 2.0f * dt;
        if (bias_track > 0.05f) {
            bias_track = 0.05f;
        }
        ekf->x[4] += bias_track * (p - ekf->x[4]);
        ekf->x[5] += bias_track * (q - ekf->x[5]);
        ekf->x[6] += bias_track * (r - ekf->x[6]);
    }

    // P = Pp - K*H*Pp;
    // K*H
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            KH[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_MEAS_DIM; k++) {
                KH[i][j] += ekf->K[i][k] * H[k][j];
            }
        }
    }

    // K*H*Pp
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            KHPp[i][j] = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_DIM; k++) {
                KHPp[i][j] += KH[i][k] * ekf->Pp[k][j];
            }
        }
    }

    // P = Pp - K*H*Pp
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] = ekf->Pp[i][j] - KHPp[i][j];
        }
    }

    EKFquaternionToEuler(ekf->x, euler);
}

uint8_t inverse_matrix(float a[6][6], float a_inv[6][6])
{
    // Initialize matrix
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            if (i != j)
                a_inv[i][j] = 0;
            else
                a_inv[i][j] = 1;
        }
    }

    // Applying Gauss Jordan Elimination
    float ratio = 1.0;
    for (uint8_t i = 0; i < 6; i++) {
        if (a[i][i] == 0.0) {
            return 0;
        }
        for (uint8_t j = 0; j < 6; j++) {
            if (i != j) {
                ratio = a[j][i] / a[i][i];
                for (uint8_t k = 0; k < 6; k++) {
                    a[j][k]     = a[j][k] - ratio * a[i][k];
                    a_inv[j][k] = a_inv[j][k] - ratio * a_inv[i][k];
                }
            }
        }
    }
    // Row Operation to Make Principal Diagonal to 1
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            a_inv[i][j] = a_inv[i][j] / a[i][i];
        }
    }
    return 1;
}

void EKFquaternionToEuler(float q[4], float euler[3])
{
    float phi = atan2f(2 * (q[2] * q[3] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));

    float sinp  = 2 * (q[1] * q[3] - q[0] * q[2]);
    float theta = 0.0;

    if (fabsf(sinp) >= 1) {
        theta = (sinp >= 0) ? M_PI / 2 : -M_PI / 2;
    } else {
        theta = -asinf(sinp);
    }

    float psi = atan2f(2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

    euler[0] = phi;
    euler[1] = theta;
    euler[2] = psi;
}
