/*
 * EKF.h
 *
 *  Created on: Apr 6, 2023
 *      Author: Emre Emir Fidan
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include "main.h"
#include "soc.h"
#include <math.h>

#define EKF_STATE_DIM           7
#define EKF_MEAS_DIM            6

#define EKF_ACCEL_TILT_LOW_RAD  0.05f
#define EKF_ACCEL_TILT_HIGH_RAD 0.30f
#define EKF_ACCEL_R_SCALE_MAX   500.0f
#define EKF_ACCEL_RELIAB_GATE   0.9f

#define EKF_STATIC_ACCEL_ERR_TH 0.15f
#define EKF_STATIC_GYRO_NORM_TH 0.35f

#define EKF_BIAS_TRACK_GAIN     2.0f
#define EKF_BIAS_TRACK_GAIN_MAX 0.05f

typedef struct
{
    float P[EKF_STATE_DIM][EKF_STATE_DIM]; // Prediction error covariance matrix
    float Q[EKF_STATE_DIM][EKF_STATE_DIM]; // Process noise covariance matrix
    float R[EKF_MEAS_DIM][EKF_MEAS_DIM];   // Measurement noise covariance matrix

    float K[EKF_STATE_DIM][EKF_MEAS_DIM];

    float x[EKF_STATE_DIM];
    float xp[EKF_STATE_DIM];

    float Pp[EKF_STATE_DIM][EKF_STATE_DIM];

    // Magnetic Vector References
    float   ref_mx;
    float   ref_my;
    float   ref_mz;
    uint8_t use_mag;
} ekf_t;

void EKF_init(ekf_t *ekf, float ref_mx, float ref_my, float ref_mz, float N_Q, float N_Q_bias, float N_P, float N_R, uint8_t use_mag);

void EKF_update(ekf_t *ekf, float euler[3], float ax, float ay, float az, float p, float q, float r, float mx, float my, float mz, float dt);

uint8_t inverse_matrix(float a[6][6], float a_inv[6][6]);

void EKFquaternionToEuler(float q[4], float euler[3]);

#endif /* INC_EKF_H_ */
