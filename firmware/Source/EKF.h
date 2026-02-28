#ifndef EKF_H
#define EKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 外部可调用的初始化函数声明 */
void EKF_init(const float nGyro, const float nAcc, const float nMag);

/* 外部可调用的核心姿态更新函数声明 */
void updateEKFQuatAtt(
    const float gyr_rps[3], const float acc_mps2[3], const float mag_unit[3], float Va_mps, float magDecRad, float T, float NdivT, float *roll_deg, float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */