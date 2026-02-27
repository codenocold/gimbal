/**
 * @file ecl_attitude_ekf.c
 * @brief ECL Attitude EKF - Implementation (C Language)
 *
 * 基于PX4 ECL EKF算法提取的独立姿态估计库实现（C语言版本）
 * 包含扩展卡尔曼滤波器的预测和校正步骤
 * 支持外力加速度抑制功能
 *
 * Copyright (c) 2024 PX4 Development Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ecl_attitude_ekf.h"
#include <string.h>

// ============================================================================
// 常量定义
// ============================================================================

#define P_MIN          1e-6f
#define P_MAX          1.0f
#define INNOVATION_MIN 1e-6f
#define INNOVATION_MAX 1.0f

// ============================================================================
// 工具函数
// ============================================================================

static float constrain_float(float val, float min_val, float max_val)
{
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// 协方差矩阵元素访问宏
#define P(ekf, i, j) ((ekf)->P[i][j])

// ============================================================================
// 初始化函数
// ============================================================================

void ecl_config_set_default(EclConfig *config)
{
    memset(config, 0, sizeof(EclConfig));

    // 噪声参数
    config->gyro_noise        = 0.01f;
    config->accel_noise       = 0.1f;
    config->mag_heading_noise = 0.3f;
    config->accel_bias_noise  = 0.001f;

    // 初始不确定性
    config->initial_attitude_std   = 0.5f;
    config->initial_gyro_bias_std  = 0.1f;
    config->initial_accel_bias_std = 0.1f;

    // 功能开关
    config->enable_accel_bias_est = 1;

    // 外力加速度抑制参数
    config->external_accel_threshold   = 2.0f; // m/s^2
    config->accel_correction_threshold = 0.5f;

    // 磁偏角
    config->mag_declination = 0.0f;

    // 最大时间步长
    config->dt_max = 0.05f; // 50ms
}

void ecl_ekf_init(EclAttitudeEKF *ekf, const EclConfig *config)
{
    memset(ekf, 0, sizeof(EclAttitudeEKF));

    // 复制配置
    memcpy(&ekf->config, config, sizeof(EclConfig));

    // 初始化状态
    ekf->state.q          = ecl_Quaternion_identity();
    ekf->state.gyro_bias  = ecl_Vector3f_zero();
    ekf->state.accel_bias = ecl_Vector3f_zero();

    // 初始化协方差矩阵为零
    memset(ekf->P, 0, sizeof(ekf->P));

    // 初始化其他变量
    ekf->velocity_ned     = ecl_Vector3f_zero();
    ekf->external_accel   = ecl_Vector3f_zero();
    ekf->accel_innovation = ecl_Vector3f_zero();
}

void ecl_ekf_destroy(EclAttitudeEKF *ekf)
{
    // C版本不需要释放动态内存
    (void) ekf;
}

// ============================================================================
// 协方差矩阵初始化
// ============================================================================

static void init_covariance(EclAttitudeEKF *ekf)
{
    // 四元数的初始不确定性
    float p_att  = ekf->config.initial_attitude_std * ekf->config.initial_attitude_std;
    P(ekf, 0, 0) = p_att; // q0
    P(ekf, 1, 1) = p_att; // q1
    P(ekf, 2, 2) = p_att; // q2
    P(ekf, 3, 3) = p_att; // q3

    // 陀螺仪零偏的初始不确定性
    float p_gyro_bias = ekf->config.initial_gyro_bias_std * ekf->config.initial_gyro_bias_std;
    P(ekf, 4, 4)      = p_gyro_bias; // bw_x
    P(ekf, 5, 5)      = p_gyro_bias; // bw_y
    P(ekf, 6, 6)      = p_gyro_bias; // bw_z

    // 加速度计零偏的初始不确定性
    if (ekf->config.enable_accel_bias_est) {
        float p_accel_bias = ekf->config.initial_accel_bias_std * ekf->config.initial_accel_bias_std;
        P(ekf, 7, 7)       = p_accel_bias; // ba_x
        P(ekf, 8, 8)       = p_accel_bias; // ba_y
        P(ekf, 9, 9)       = p_accel_bias; // ba_z
    }
}

// ============================================================================
// 启动函数
// ============================================================================

int ecl_ekf_start(EclAttitudeEKF *ekf, uint64_t timestamp_us, const float initial_accel[3])
{
    ekf->timestamp_us       = timestamp_us;
    ekf->last_imu_update_us = timestamp_us;
    ekf->dt                 = 0.0f;

    // 如果提供了初始加速度计数据，使用它来初始化姿态
    if (initial_accel != NULL) {
        ecl_Vector3f accel        = ecl_Vector3f_make(initial_accel[0], initial_accel[1], initial_accel[2]);
        float        accel_length = ecl_Vector3f_length(&accel);

        if (accel_length > 1.0f) {
            // 计算俯仰角和横滚角
            float pitch = asinf(accel.v[0] / ECL_MATH_GRAVITY);
            float roll  = atan2f(-accel.v[1], -accel.v[2]);

            // 初始化四元数
            ekf->state.q = ecl_Quaternion_fromEuler(roll, pitch, 0.0f);
        }
    } else {
        // 默认姿态：水平
        ekf->state.q = ecl_Quaternion_identity();
    }

    // 初始化协方差矩阵
    init_covariance(ekf);

    ekf->initialized = 1;
    return 1;
}

void ecl_ekf_reset(EclAttitudeEKF *ekf)
{
    ekf->initialized        = 0;
    ekf->timestamp_us       = 0;
    ekf->last_imu_update_us = 0;
    ekf->dt                 = 0.0f;

    ekf->state.q          = ecl_Quaternion_identity();
    ekf->state.gyro_bias  = ecl_Vector3f_zero();
    ekf->state.accel_bias = ecl_Vector3f_zero();

    memset(ekf->P, 0, sizeof(ekf->P));

    ekf->new_imu_data            = 0;
    ekf->new_mag_data            = 0;
    ekf->external_accel_detected = 0;
    ekf->external_accel          = ecl_Vector3f_zero();
    ekf->velocity_ned            = ecl_Vector3f_zero();
    ekf->velocity_initialized    = 0;
}

// ============================================================================
// 数据输入接口
// ============================================================================

void ecl_ekf_set_imu_data(EclAttitudeEKF *ekf, const float gyro[3], const float accel[3], uint64_t timestamp_us)
{
    ecl_Vector3f_set(&ekf->imu_gyro, gyro[0], gyro[1], gyro[2]);
    ecl_Vector3f_set(&ekf->imu_accel, accel[0], accel[1], accel[2]);
    ekf->timestamp_us = timestamp_us;
    ekf->new_imu_data = 1;
}

void ecl_ekf_set_mag_data(EclAttitudeEKF *ekf, const float mag[3], uint64_t timestamp_us)
{
    ecl_Vector3f_set(&ekf->mag_data, mag[0], mag[1], mag[2]);
    ekf->last_mag_update_us = timestamp_us;
    ekf->new_mag_data       = 1;
}

void ecl_ekf_enable_mag_fusion(EclAttitudeEKF *ekf, int enable)
{
    ekf->mag_fusion_enabled = enable;
}

// ============================================================================
// 辅助函数
// ============================================================================

// 校正陀螺仪数据（去除零偏）
static ecl_Vector3f correct_gyro(EclAttitudeEKF *ekf, const ecl_Vector3f *gyro)
{
    return ecl_Vector3f_sub(gyro, &ekf->state.gyro_bias);
}

// 校正加速度计数据（去除零偏）
static ecl_Vector3f apply_accel_bias_correction(EclAttitudeEKF *ekf, const ecl_Vector3f *accel)
{
    return ecl_Vector3f_sub(accel, &ekf->state.accel_bias);
}

// 计算期望的重力向量（在机体系）
static ecl_Vector3f compute_expected_accel(const ecl_Quaternion *q)
{
    // 期望的重力向量：在NED坐标系中为[0, 0, -g]
    // 转换到机体系
    ecl_Vector3f gravity_ned = ecl_Vector3f_make(0.0f, 0.0f, -ECL_MATH_GRAVITY);
    return ecl_Quaternion_rotateVector(q, &gravity_ned);
}

// 计算期望的磁场向量（在机体系）
static ecl_Vector3f compute_expected_mag(const ecl_Quaternion *q, float mag_declination)
{
    // 磁场向量分解
    float mag_north = 1.0f; // 归一化北向分量
    float mag_east  = 0.0f; // 归一化东向分量
    float mag_down  = tanf(mag_declination) * mag_north;

    // 转换到机体系
    ecl_Vector3f mag_ned = ecl_Vector3f_make(mag_north, mag_east, mag_down);
    ecl_Vector3f_normalize_inplace(&mag_ned);

    return ecl_Quaternion_rotateVector(q, &mag_ned);
}

// 计算四元数导数
static ecl_Quaternion compute_quat_derivative(const ecl_Vector3f *omega, const ecl_Quaternion *q)
{
    float w = q->q[0], x = q->q[1], y = q->q[2], z = q->q[3];
    float wx = omega->v[0], wy = omega->v[1], wz = omega->v[2];

    return ecl_Quaternion_make(0.5f * (-x * wx - y * wy - z * wz), 0.5f * (w * wx + y * wz - z * wy), 0.5f * (w * wy - x * wz + z * wx), 0.5f * (w * wz + x * wy - y * wx));
}

// 外力加速度检测
static void detect_external_accel(EclAttitudeEKF *ekf, const ecl_Vector3f *measured_accel, const ecl_Vector3f *expected_accel)
{
    // 计算加速度 innovation magnitude
    ecl_Vector3f diff           = ecl_Vector3f_sub(measured_accel, expected_accel);
    float        innovation_mag = ecl_Vector3f_length(&diff);

    // 检测阈值
    float threshold = ekf->config.external_accel_threshold;

    if (innovation_mag > threshold) {
        // 检测到外力加速度
        ekf->external_accel_detected = 1;

        // 估计外力加速度（在机体系）
        ekf->external_accel = diff;
    } else {
        // 无外力加速度
        ekf->external_accel_detected = 0;
        ekf->external_accel          = ecl_Vector3f_zero();
    }
}

// 速度更新（用于运动补偿）
static void update_velocity(EclAttitudeEKF *ekf, const ecl_Vector3f *accel_ned, float dt)
{
    if (!ekf->velocity_initialized) {
        // 第一次初始化速度
        ekf->velocity_ned         = ecl_Vector3f_scale(accel_ned, dt);
        ekf->velocity_initialized = 1;
    } else {
        // 积分加速度得到速度
        ecl_Vector3f delta_v = ecl_Vector3f_scale(accel_ned, dt);
        ekf->velocity_ned    = ecl_Vector3f_add(&ekf->velocity_ned, &delta_v);
    }
}

// 归一化四元数
static void normalize_quaternion(ecl_Quaternion *q)
{
    ecl_Quaternion_normalize_inplace(q);
}

// ============================================================================
// 预测步骤
// ============================================================================

static void predict(EclAttitudeEKF *ekf, float dt)
{
    // 校正陀螺仪数据（去除零偏）
    ecl_Vector3f omega = correct_gyro(ekf, &ekf->imu_gyro);

    // 计算四元数导数
    ecl_Quaternion q_dot = compute_quat_derivative(&omega, &ekf->state.q);

    // 积分四元数 (一阶欧拉法)
    ekf->state.q.q[0] += q_dot.q[0] * dt;
    ekf->state.q.q[1] += q_dot.q[1] * dt;
    ekf->state.q.q[2] += q_dot.q[2] * dt;
    ekf->state.q.q[3] += q_dot.q[3] * dt;

    // 归一化四元数
    normalize_quaternion(&ekf->state.q);

    // 更新速度估计（用于运动补偿）
    ecl_Vector3f corrected_accel = apply_accel_bias_correction(ekf, &ekf->imu_accel);
    ecl_Vector3f accel_ned       = ecl_Quaternion_rotateVector(&ekf->state.q, &corrected_accel);
    // 添加重力
    accel_ned.v[2] += ECL_MATH_GRAVITY;
    update_velocity(ekf, &accel_ned, dt);

    // 过程噪声
    float dt2 = dt * dt;

    // 陀螺仪噪声影响
    float q_gyro = ekf->config.gyro_noise * dt2;
    // 加速度计噪声影响
    float q_accel = ekf->config.accel_noise * dt2;
    // 加速度计零偏随机游走噪声
    float q_accel_bias = ekf->config.accel_bias_noise * dt2;

    // 更新协方差矩阵
    for (int i = 0; i < 4; i++) {
        P(ekf, i, i) += q_accel;
    }
    for (int i = 4; i < 7; i++) {
        P(ekf, i, i) += q_gyro;
    }
    // 加速度计零偏协方差更新
    if (ekf->config.enable_accel_bias_est) {
        for (int i = 7; i < 10; i++) {
            P(ekf, i, i) += q_accel_bias;
        }
    }

    // 限制协方差范围
    for (int i = 0; i < 10; i++) {
        P(ekf, i, i) = constrain_float(P(ekf, i, i), P_MIN, P_MAX);
    }
}

// ============================================================================
// 加速度计校正（带外力加速度抑制）
// ============================================================================

static void correct_accel(EclAttitudeEKF *ekf)
{
    // 校正加速度计数据（去除零偏）
    ecl_Vector3f corrected_accel = apply_accel_bias_correction(ekf, &ekf->imu_accel);

    // 计算期望的重力向量（在机体系）
    ecl_Vector3f expected_accel = compute_expected_accel(&ekf->state.q);

    // 测量向量（机体系下的重力）
    ecl_Vector3f measured_accel = corrected_accel;
    ecl_Vector3f_normalize_inplace(&measured_accel);
    measured_accel = ecl_Vector3f_scale(&measured_accel, ECL_MATH_GRAVITY);

    // 计算创新向量 (测量 - 预测)
    ecl_Vector3f innovation = ecl_Vector3f_sub(&measured_accel, &expected_accel);
    ekf->accel_innovation   = innovation;

    // 检测外力加速度
    detect_external_accel(ekf, &corrected_accel, &expected_accel);

    // 如果检测到外力加速度，减小校正权重
    float correction_weight = 1.0f;
    if (ekf->external_accel_detected) {
        float innovation_len = ecl_Vector3f_length(&ekf->accel_innovation);
        correction_weight    = ekf->config.accel_correction_threshold / (innovation_len + 0.1f);
        correction_weight    = constrain_float(correction_weight, 0.0f, 1.0f);
    }

    if (correction_weight < 0.01f) {
        // 如果校正权重太小，跳过校正
        return;
    }

    // 简化的协方差计算
    float innov_cov = P(ekf, 0, 0) + P(ekf, 1, 1) + P(ekf, 2, 2) + ekf->config.accel_noise;
    innov_cov       = constrain_float(innov_cov, INNOVATION_MIN, INNOVATION_MAX);

    // 卡尔曼增益
    float K[10][3];
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = P(ekf, i, j) / innov_cov;
        }
    }

    // 更新状态
    for (int i = 0; i < 4; i++) {
        float delta_q = (K[i][0] * innovation.v[0] + K[i][1] * innovation.v[1] + K[i][2] * innovation.v[2]) * correction_weight;

        if (i == 0) {
            ekf->state.q.q[0] -= delta_q * 0.5f;
        } else {
            ekf->state.q.q[i] += delta_q * 0.5f;
        }
    }

    // 如果启用加速度计零偏估计，更新零偏
    if (ekf->config.enable_accel_bias_est) {
        // 加速度计零偏的观测模型：直接影响加速度测量
        // 简化的零偏更新
        float K_accel[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K_accel[i][j] = P(ekf, 7 + i, j) / innov_cov;
            }
        }

        // 更新加速度计零偏
        for (int i = 0; i < 3; i++) {
            float delta_bias = (K_accel[i][0] * innovation.v[0] + K_accel[i][1] * innovation.v[1] + K_accel[i][2] * innovation.v[2]) * correction_weight;
            ekf->state.accel_bias.v[i] += delta_bias;
        }
    }

    // 更新协方差矩阵
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            float KH = 0.0f;
            for (int k = 0; k < 3; k++) {
                int condition = (j < 4) || (j >= 7 && ekf->config.enable_accel_bias_est);
                KH += K[i][k] * (condition ? 1.0f : 0.0f);
            }
            P(ekf, i, j) = P(ekf, i, j) - KH * P(ekf, j, j) * correction_weight;
        }
    }

    // 限制协方差
    for (int i = 0; i < 10; i++) {
        P(ekf, i, i) = constrain_float(P(ekf, i, i), P_MIN, P_MAX);
    }
}

// ============================================================================
// 磁力计校正（航向校正）
// ============================================================================

static void correct_mag(EclAttitudeEKF *ekf)
{
    // 计算期望的磁场向量（在机体系）
    ecl_Vector3f expected_mag = compute_expected_mag(&ekf->state.q, ekf->config.mag_declination);

    // 归一化测量值
    ecl_Vector3f measured_mag = ecl_Vector3f_normalize(&ekf->mag_data);

    // 计算创新向量
    ecl_Vector3f innovation = ecl_Vector3f_sub(&measured_mag, &expected_mag);

    // 创新协方差
    float innov_cov = P(ekf, 2, 2) + P(ekf, 3, 3) + ekf->config.mag_heading_noise;
    innov_cov       = constrain_float(innov_cov, INNOVATION_MIN, INNOVATION_MAX);

    // 卡尔曼增益
    float K[10][2];
    for (int i = 0; i < 10; i++) {
        K[i][0] = P(ekf, i, 2) / innov_cov;
        K[i][1] = P(ekf, i, 3) / innov_cov;
    }

    // 更新状态
    for (int i = 0; i < 4; i++) {
        float delta_q = K[i][0] * innovation.v[0] + K[i][1] * innovation.v[1];

        if (i == 0) {
            ekf->state.q.q[0] -= delta_q * 0.5f;
        } else {
            ekf->state.q.q[i] += delta_q * 0.5f;
        }
    }

    // 更新协方差
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            float KH     = K[i][0] * ((j == 2) ? 1.0f : 0.0f) + K[i][1] * ((j == 3) ? 1.0f : 0.0f);
            P(ekf, i, j) = P(ekf, i, j) - KH * P(ekf, j, j);
        }
    }

    // 限制协方差
    for (int i = 0; i < 10; i++) {
        P(ekf, i, i) = constrain_float(P(ekf, i, i), P_MIN, P_MAX);
    }
}

// ============================================================================
// 主更新循环
// ============================================================================

int ecl_ekf_update(EclAttitudeEKF *ekf)
{
    if (!ekf->initialized) {
        return 0;
    }

    // 计算时间步长
    if (ekf->last_imu_update_us > 0) {
        ekf->dt = (float) (ekf->timestamp_us - ekf->last_imu_update_us) / 1e6f;
    } else {
        ekf->dt = 0.01f; // 默认10ms
    }

    // 限制时间步长
    ekf->dt = constrain_float(ekf->dt, 0.0f, ekf->config.dt_max);

    // 检查是否有新数据
    if (!ekf->new_imu_data) {
        return 0;
    }

    // 预测步骤
    predict(ekf, ekf->dt);

    // 加速度计校正步骤（倾斜校正，带外力加速度抑制）
    correct_accel(ekf);

    // 磁力计校正步骤（航向校正）
    if (ekf->mag_fusion_enabled && ekf->new_mag_data) {
        correct_mag(ekf);
    }

    // 归一化四元数
    normalize_quaternion(&ekf->state.q);

    // 更新状态
    ekf->last_imu_update_us = ekf->timestamp_us;
    ekf->new_imu_data       = 0;
    ekf->new_mag_data       = 0;

    return 1;
}

// ============================================================================
// 输出接口
// ============================================================================

void ecl_ekf_get_euler_angles(const EclAttitudeEKF *ekf, float *roll, float *pitch, float *yaw)
{
    ecl_Quaternion_toEuler(&ekf->state.q, roll, pitch, yaw);
}

void ecl_ekf_get_quaternion(const EclAttitudeEKF *ekf, float q[4])
{
    q[0] = ekf->state.q.q[0];
    q[1] = ekf->state.q.q[1];
    q[2] = ekf->state.q.q[2];
    q[3] = ekf->state.q.q[3];
}

void ecl_ekf_get_gyro_bias(const EclAttitudeEKF *ekf, float bias[3])
{
    bias[0] = ekf->state.gyro_bias.v[0];
    bias[1] = ekf->state.gyro_bias.v[1];
    bias[2] = ekf->state.gyro_bias.v[2];
}

void ecl_ekf_get_accel_bias(const EclAttitudeEKF *ekf, float bias[3])
{
    bias[0] = ekf->state.accel_bias.v[0];
    bias[1] = ekf->state.accel_bias.v[1];
    bias[2] = ekf->state.accel_bias.v[2];
}

int ecl_ekf_get_external_accel_detected(const EclAttitudeEKF *ekf)
{
    return ekf->external_accel_detected;
}

void ecl_ekf_get_accel_innovation(const EclAttitudeEKF *ekf, float innovation[3])
{
    innovation[0] = ekf->accel_innovation.v[0];
    innovation[1] = ekf->accel_innovation.v[1];
    innovation[2] = ekf->accel_innovation.v[2];
}

int ecl_ekf_is_initialized(const EclAttitudeEKF *ekf)
{
    return ekf->initialized;
}
