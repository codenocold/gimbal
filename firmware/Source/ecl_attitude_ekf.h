/**
 * @file ecl_attitude_ekf.h
 * @brief ECL Attitude EKF (C Language)
 *
 * 基于PX4 ECL EKF算法提取的独立姿态估计库实现（C语言版本）
 * 包含扩展卡尔曼滤波器的预测和校正步骤
 * 支持外力加速度抑制功能
 *
 * Copyright (c) 2024 PX4 Development Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ECL_ATTITUDE_EKF_H
#define ECL_ATTITUDE_EKF_H

#include "ecl_attitude_ekf_math.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 配置结构体
// ============================================================================

typedef struct
{
    // 噪声参数
    float gyro_noise;        // 陀螺仪噪声
    float accel_noise;       // 加速度计噪声
    float mag_heading_noise; // 磁力计航向噪声
    float accel_bias_noise;  // 加速度计零偏随机游走噪声

    // 初始状态不确定性
    float initial_attitude_std;   // 初始姿态标准差 (rad)
    float initial_gyro_bias_std;  // 初始陀螺仪零偏标准差 (rad/s)
    float initial_accel_bias_std; // 初始加速度计零偏标准差 (m/s^2)

    // 功能开关
    int enable_accel_bias_est; // 是否启用加速度计零偏估计

    // 外力加速度抑制参数
    float external_accel_threshold;   // 外力加速度检测阈值 (m/s^2)
    float accel_correction_threshold; // 加速度校正权重阈值

    // 磁偏角 (rad)
    float mag_declination;

    // 最大时间步长 (s)
    float dt_max;
} EclConfig;

// 默认配置
void ecl_config_set_default(EclConfig *config);

// ============================================================================
// EKF状态结构体
// ============================================================================

// 状态向量维度
#define ECL_EKF_STATE_DIM 10

// 状态: [q0, q1, q2, q3, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z]
// 四元数(4) + 陀螺仪零偏(3) + 加速度计零偏(3)

typedef struct
{
    ecl_Quaternion q;          // 姿态四元数
    ecl_Vector3f   gyro_bias;  // 陀螺仪零偏 (rad/s)
    ecl_Vector3f   accel_bias; // 加速度计零偏 (m/s^2)
} EclState;

// EKF实例结构体
typedef struct
{
    // 配置
    EclConfig config;

    // 状态
    EclState state;

    // 协方差矩阵 (10x10)
    float P[ECL_EKF_STATE_DIM][ECL_EKF_STATE_DIM];

    // IMU数据
    ecl_Vector3f imu_gyro;
    ecl_Vector3f imu_accel;

    // 磁力计数据
    ecl_Vector3f mag_data;

    // 创新向量
    ecl_Vector3f accel_innovation;

    // 速度 (NED坐标系) - 用于运动补偿
    ecl_Vector3f velocity_ned;

    // 外力加速度
    ecl_Vector3f external_accel;
    int          external_accel_detected;

    // 状态标志
    int initialized;
    int mag_fusion_enabled;

    // 时间戳
    uint64_t timestamp_us;
    uint64_t last_imu_update_us;
    uint64_t last_mag_update_us;

    // 时间步长
    float dt;

    // 新数据标志
    int new_imu_data;
    int new_mag_data;

    // 速度初始化标志
    int velocity_initialized;
} EclAttitudeEKF;

// ============================================================================
// EKF函数接口
// ============================================================================

/**
 * @brief 创建EKF实例
 */
void ecl_ekf_init(EclAttitudeEKF *ekf, const EclConfig *config);

/**
 * @brief 初始化EKF
 * @param ekf EKF实例
 * @param timestamp_us 初始时间戳 (微秒)
 * @param initial_accel 初始加速度计数据 [ax, ay, az] (可为NULL)
 * @return 成功返回1，失败返回0
 */
int ecl_ekf_start(EclAttitudeEKF *ekf, uint64_t timestamp_us, const float initial_accel[3]);

/**
 * @brief 重置EKF
 */
void ecl_ekf_reset(EclAttitudeEKF *ekf);

/**
 * @brief 销毁EKF实例
 */
void ecl_ekf_destroy(EclAttitudeEKF *ekf);

/**
 * @brief 设置IMU数据
 * @param gyro 陀螺仪数据 [wx, wy, wz] (rad/s)
 * @param accel 加速度计数据 [ax, ay, az] (m/s^2)
 * @param timestamp_us 时间戳 (微秒)
 */
void ecl_ekf_set_imu_data(EclAttitudeEKF *ekf, const float gyro[3], const float accel[3], uint64_t timestamp_us);

/**
 * @brief 设置磁力计数据
 * @param mag 磁力计数据 [mx, my, mz]
 * @param timestamp_us 时间戳 (微秒)
 */
void ecl_ekf_set_mag_data(EclAttitudeEKF *ekf, const float mag[3], uint64_t timestamp_us);

/**
 * @brief 启用/禁用磁力计融合
 */
void ecl_ekf_enable_mag_fusion(EclAttitudeEKF *ekf, int enable);

/**
 * @brief 运行EKF更新
 * @return 成功返回1，失败返回0
 */
int ecl_ekf_update(EclAttitudeEKF *ekf);

// ============================================================================
// 输出接口
// ============================================================================

/**
 * @brief 获取欧拉角
 * @param roll 俯仰角输出 (rad)
 * @param pitch 横滚角输出 (rad)
 * @param yaw 航向角输出 (rad)
 */
void ecl_ekf_get_euler_angles(const EclAttitudeEKF *ekf, float *roll, float *pitch, float *yaw);

/**
 * @brief 获取四元数
 * @param q 四元数输出数组 [w, x, y, z]
 */
void ecl_ekf_get_quaternion(const EclAttitudeEKF *ekf, float q[4]);

/**
 * @brief 获取陀螺仪零偏
 * @param bias 零偏输出数组 [bx, by, bz] (rad/s)
 */
void ecl_ekf_get_gyro_bias(const EclAttitudeEKF *ekf, float bias[3]);

/**
 * @brief 获取加速度计零偏
 * @param bias 零偏输出数组 [bx, by, bz] (m/s^2)
 */
void ecl_ekf_get_accel_bias(const EclAttitudeEKF *ekf, float bias[3]);

/**
 * @brief 获取外力加速度检测状态
 * @return 1表示检测到外力加速度，0表示未检测到
 */
int ecl_ekf_get_external_accel_detected(const EclAttitudeEKF *ekf);

/**
 * @brief 获取加速度 innovation
 * @param innovation innovation输出数组
 */
void ecl_ekf_get_accel_innovation(const EclAttitudeEKF *ekf, float innovation[3]);

/**
 * @brief 检查EKF是否已初始化
 * @return 1表示已初始化，0表示未初始化
 */
int ecl_ekf_is_initialized(const EclAttitudeEKF *ekf);

#ifdef __cplusplus
}
#endif

#endif /* ECL_ATTITUDE_EKF_H */
