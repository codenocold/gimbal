/**
 * @file ecl_attitude_ekf_math.h
 * @brief ECL Attitude EKF - Math Utilities (C Language)
 *
 * C语言版本的数学工具库
 * 包含向量、四元数、矩阵运算
 *
 * Copyright (c) 2024 PX4 Development Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ECL_ATTITUDE_EKF_MATH_H
#define ECL_ATTITUDE_EKF_MATH_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 常量定义
// ============================================================================

#define ECL_MATH_PI                  3.14159265358979323846f
#define ECL_MATH_PI_2                1.57079632679489661923f

// 地球自转速率 (rad/s)
#define ECL_MATH_EARTH_ROTATION_RATE 7.2921159e-5f

// 重力加速度 (m/s^2)
#define ECL_MATH_GRAVITY             9.80665f

// ============================================================================
// 工具函数
// ============================================================================

// 限制数值范围
static inline float ecl_constrain(float val, float min_val, float max_val)
{
    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

// 平方
static inline float ecl_sqr(float val)
{
    return val * val;
}

// ============================================================================
// 3D向量结构体
// ============================================================================

typedef struct
{
    float v[3];
} ecl_Vector3f;

// 向量初始化
static inline ecl_Vector3f ecl_Vector3f_make(float x, float y, float z)
{
    ecl_Vector3f result;
    result.v[0] = x;
    result.v[1] = y;
    result.v[2] = z;
    return result;
}

// 零向量
static inline ecl_Vector3f ecl_Vector3f_zero(void)
{
    return ecl_Vector3f_make(0.0f, 0.0f, 0.0f);
}

// 向量加法
static inline ecl_Vector3f ecl_Vector3f_add(const ecl_Vector3f *a, const ecl_Vector3f *b)
{
    return ecl_Vector3f_make(a->v[0] + b->v[0], a->v[1] + b->v[1], a->v[2] + b->v[2]);
}

// 向量减法
static inline ecl_Vector3f ecl_Vector3f_sub(const ecl_Vector3f *a, const ecl_Vector3f *b)
{
    return ecl_Vector3f_make(a->v[0] - b->v[0], a->v[1] - b->v[1], a->v[2] - b->v[2]);
}

// 向量数乘
static inline ecl_Vector3f ecl_Vector3f_scale(const ecl_Vector3f *a, float scale)
{
    return ecl_Vector3f_make(a->v[0] * scale, a->v[1] * scale, a->v[2] * scale);
}

// 向量点积
static inline float ecl_Vector3f_dot(const ecl_Vector3f *a, const ecl_Vector3f *b)
{
    return a->v[0] * b->v[0] + a->v[1] * b->v[1] + a->v[2] * b->v[2];
}

// 向量叉积
static inline ecl_Vector3f ecl_Vector3f_cross(const ecl_Vector3f *a, const ecl_Vector3f *b)
{
    return ecl_Vector3f_make(a->v[1] * b->v[2] - a->v[2] * b->v[1], a->v[2] * b->v[0] - a->v[0] * b->v[2], a->v[0] * b->v[1] - a->v[1] * b->v[0]);
}

// 向量模长
static inline float ecl_Vector3f_length(const ecl_Vector3f *a)
{
    return sqrtf(a->v[0] * a->v[0] + a->v[1] * a->v[1] + a->v[2] * a->v[2]);
}

// 向量归一化
static inline ecl_Vector3f ecl_Vector3f_normalize(const ecl_Vector3f *a)
{
    float len = ecl_Vector3f_length(a);
    if (len > 1e-6f) {
        return ecl_Vector3f_scale(a, 1.0f / len);
    }
    return ecl_Vector3f_zero();
}

// 原地归一化
static inline void ecl_Vector3f_normalize_inplace(ecl_Vector3f *a)
{
    float len = ecl_Vector3f_length(a);
    if (len > 1e-6f) {
        a->v[0] /= len;
        a->v[1] /= len;
        a->v[2] /= len;
    }
}

// 设置向量值
static inline void ecl_Vector3f_set(ecl_Vector3f *a, float x, float y, float z)
{
    a->v[0] = x;
    a->v[1] = y;
    a->v[2] = z;
}

// 复制向量
static inline void ecl_Vector3f_copy(ecl_Vector3f *dst, const ecl_Vector3f *src)
{
    dst->v[0] = src->v[0];
    dst->v[1] = src->v[1];
    dst->v[2] = src->v[2];
}

// ============================================================================
// 四元数结构体
// ============================================================================

typedef struct
{
    float q[4]; // q[0]=w, q[1]=x, q[2]=y, q[3]=z
} ecl_Quaternion;

// 四元数初始化
static inline ecl_Quaternion ecl_Quaternion_make(float w, float x, float y, float z)
{
    ecl_Quaternion result;
    result.q[0] = w;
    result.q[1] = x;
    result.q[2] = y;
    result.q[3] = z;
    return result;
}

// 身份四元数（无旋转）
static inline ecl_Quaternion ecl_Quaternion_identity(void)
{
    return ecl_Quaternion_make(1.0f, 0.0f, 0.0f, 0.0f);
}

// 从欧拉角创建四元数 (roll, pitch, yaw in radians)
static inline ecl_Quaternion ecl_Quaternion_fromEuler(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    return ecl_Quaternion_make(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy);
}

// 四元数乘法
static inline ecl_Quaternion ecl_Quaternion_multiply(const ecl_Quaternion *a, const ecl_Quaternion *b)
{
    return ecl_Quaternion_make(a->q[0] * b->q[0] - a->q[1] * b->q[1] - a->q[2] * b->q[2] - a->q[3] * b->q[3],
                               a->q[0] * b->q[1] + a->q[1] * b->q[0] + a->q[2] * b->q[3] - a->q[3] * b->q[2],
                               a->q[0] * b->q[2] - a->q[1] * b->q[3] + a->q[2] * b->q[0] + a->q[3] * b->q[1],
                               a->q[0] * b->q[3] + a->q[1] * b->q[2] - a->q[2] * b->q[1] + a->q[3] * b->q[0]);
}

// 四元数模长
static inline float ecl_Quaternion_length(const ecl_Quaternion *q)
{
    return sqrtf(q->q[0] * q->q[0] + q->q[1] * q->q[1] + q->q[2] * q->q[2] + q->q[3] * q->q[3]);
}

// 四元数归一化
static inline ecl_Quaternion ecl_Quaternion_normalize(const ecl_Quaternion *q)
{
    float len = ecl_Quaternion_length(q);
    if (len > 1e-6f) {
        float inv_len = 1.0f / len;
        return ecl_Quaternion_make(q->q[0] * inv_len, q->q[1] * inv_len, q->q[2] * inv_len, q->q[3] * inv_len);
    }
    return ecl_Quaternion_identity();
}

// 原地归一化
static inline void ecl_Quaternion_normalize_inplace(ecl_Quaternion *q)
{
    float len = ecl_Quaternion_length(q);
    if (len > 1e-6f) {
        float inv_len = 1.0f / len;
        q->q[0] *= inv_len;
        q->q[1] *= inv_len;
        q->q[2] *= inv_len;
        q->q[3] *= inv_len;
    }
}

// 四元数共轭
static inline ecl_Quaternion ecl_Quaternion_conjugate(const ecl_Quaternion *q)
{
    return ecl_Quaternion_make(q->q[0], -q->q[1], -q->q[2], -q->q[3]);
}

// 四元数逆
static inline ecl_Quaternion ecl_Quaternion_inverse(const ecl_Quaternion *q)
{
    float len_sq = q->q[0] * q->q[0] + q->q[1] * q->q[1] + q->q[2] * q->q[2] + q->q[3] * q->q[3];
    if (len_sq > 1e-6f) {
        float inv_len_sq = 1.0f / len_sq;
        return ecl_Quaternion_make(q->q[0] * inv_len_sq, -q->q[1] * inv_len_sq, -q->q[2] * inv_len_sq, -q->q[3] * inv_len_sq);
    }
    return ecl_Quaternion_identity();
}

// 四元数旋转向量：将向量从惯性坐标系转换到机体系
static inline ecl_Vector3f ecl_Quaternion_rotateVector(const ecl_Quaternion *q, const ecl_Vector3f *v)
{
    // q * v * q^(-1), 其中 v = [0, v.x, v.y, v.z]
    float qw = q->q[0], qx = q->q[1], qy = q->q[2], qz = q->q[3];
    float vx = v->v[0], vy = v->v[1], vz = v->v[2];

    // 计算 q * v
    float qv_x = qw * vx + qy * vz - qz * vy;
    float qv_y = qw * vy + qz * vx - qx * vz;
    float qv_z = qw * vz + qx * vy - qy * vx;
    float qv_w = -qx * vx - qy * vy - qz * vz;

    // 计算 (q * v) * q^(-1)
    float rx = qv_x * qw - qv_w * qx - qv_y * qz + qv_z * qy;
    float ry = qv_y * qw - qv_w * qy - qv_z * qx + qv_x * qz;
    float rz = qv_z * qw - qv_w * qz - qv_x * qy + qv_x * qx;

    return ecl_Vector3f_make(rx, ry, rz);
}

// 从四元数获取欧拉角 (roll, pitch, yaw in radians)
static inline void ecl_Quaternion_toEuler(const ecl_Quaternion *q, float *roll, float *pitch, float *yaw)
{
    float qw = q->q[0], qx = q->q[1], qy = q->q[2], qz = q->q[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll           = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(ECL_MATH_PI_2, sinp); // use 90 degrees if out of range
    } else {
        *pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw            = atan2f(siny_cosp, cosy_cosp);
}

// 复制四元数
static inline void ecl_Quaternion_copy(ecl_Quaternion *dst, const ecl_Quaternion *src)
{
    dst->q[0] = src->q[0];
    dst->q[1] = src->q[1];
    dst->q[2] = src->q[2];
    dst->q[3] = src->q[3];
}

// ============================================================================
// 3x3矩阵结构体
// ============================================================================

typedef struct
{
    float m[3][3];
} ecl_Matrix3x3;

// 矩阵初始化（行优先）
static inline ecl_Matrix3x3 ecl_Matrix3x3_make(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
{
    ecl_Matrix3x3 result;
    result.m[0][0] = m00;
    result.m[0][1] = m01;
    result.m[0][2] = m02;
    result.m[1][0] = m10;
    result.m[1][1] = m11;
    result.m[1][2] = m12;
    result.m[2][0] = m20;
    result.m[2][1] = m21;
    result.m[2][2] = m22;
    return result;
}

// 零矩阵
static inline ecl_Matrix3x3 ecl_Matrix3x3_zero(void)
{
    return ecl_Matrix3x3_make(0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// 单位矩阵
static inline ecl_Matrix3x3 ecl_Matrix3x3_identity(void)
{
    return ecl_Matrix3x3_make(1, 0, 0, 0, 1, 0, 0, 0, 1);
}

// 矩阵加法
static inline ecl_Matrix3x3 ecl_Matrix3x3_add(const ecl_Matrix3x3 *a, const ecl_Matrix3x3 *b)
{
    ecl_Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = a->m[i][j] + b->m[i][j];
        }
    }
    return result;
}

// 矩阵减法
static inline ecl_Matrix3x3 ecl_Matrix3x3_sub(const ecl_Matrix3x3 *a, const ecl_Matrix3x3 *b)
{
    ecl_Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = a->m[i][j] - b->m[i][j];
        }
    }
    return result;
}

// 矩阵乘法 (3x3 * 3x3)
static inline ecl_Matrix3x3 ecl_Matrix3x3_multiply(const ecl_Matrix3x3 *a, const ecl_Matrix3x3 *b)
{
    ecl_Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result.m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
    return result;
}

// 矩阵乘向量 (3x3 * 3x1)
static inline ecl_Vector3f ecl_Matrix3x3_multiplyVector(const ecl_Matrix3x3 *a, const ecl_Vector3f *v)
{
    ecl_Vector3f result;
    for (int i = 0; i < 3; i++) {
        result.v[i] = a->m[i][0] * v->v[0] + a->m[i][1] * v->v[1] + a->m[i][2] * v->v[2];
    }
    return result;
}

// 矩阵转置
static inline ecl_Matrix3x3 ecl_Matrix3x3_transpose(const ecl_Matrix3x3 *a)
{
    ecl_Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = a->m[j][i];
        }
    }
    return result;
}

// 矩阵标量乘法
static inline ecl_Matrix3x3 ecl_Matrix3x3_scale(const ecl_Matrix3x3 *a, float scale)
{
    ecl_Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = a->m[i][j] * scale;
        }
    }
    return result;
}

#ifdef __cplusplus
}
#endif

#endif /* ECL_ATTITUDE_EKF_MATH_H */
