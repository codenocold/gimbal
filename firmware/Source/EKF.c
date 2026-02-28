#include "EKF.h"
#include <math.h>
#include <string.h>

/* 内部全局变量定义（静态，仅本文件内有效） */
static float p, q, r;
static float ax, ay, az;
static float mx, my, mz;
static float Va;
static float lpfGyr, lpfAcc, lpfMag, lpfVa;
static float x[7];
static float P[49];
static float Q[49];
static float R[36];
static float g;

/* 初始化函数 */
void EKF_init(const float nGyro, const float nAcc, const float nMag)
{
    int   i;
    float dv4[7] = {1.0e-6f, 1.0e-6f, 1.0e-6f, 1.0e-6f, 1.0e-6f, 1.0e-6f, 1.0e-6f};
    float dv5[7] = {nGyro * nGyro, nGyro * nGyro, nGyro * nGyro, nGyro * nGyro, 1.0e-6f, 1.0e-6f, 1.0e-6f};
    float dv6[6] = {nAcc * nAcc, nAcc * nAcc, nAcc * nAcc, nMag, nMag, nMag};

    g = 9.81f;

    /* Low-pass filtered measurements */
    p  = 0.0f;
    q  = 0.0f;
    r  = 0.0f;
    ax = 0.0f;
    ay = 0.0f;
    az = 0.0f;
    mx = 0.0f;
    my = 0.0f;
    mz = 0.0f;
    Va = 0.0f;

    /* Low-pass filter coefficients */
    lpfGyr = 0.7f;
    lpfAcc = 0.9f;
    lpfMag = 0.4f;
    lpfVa  = 0.7f;

    /* Initialise state estimate vector */
    for (i = 0; i < 7; i++) {
        x[i] = 0.0f;
    }
    x[0] = 1.0f;

    /* Initialise covariance matrix */
    memset(&P[0], 0, 49U * sizeof(float));
    for (i = 0; i < 7; i++) {
        P[i + 7 * i] = dv4[i];
    }

    /* Process noise matrix */
    memset(&Q[0], 0, 49U * sizeof(float));
    for (i = 0; i < 7; i++) {
        Q[i + 7 * i] = dv5[i];
    }

    /* Measurement noise matrix */
    memset(&R[0], 0, 36U * sizeof(float));
    for (i = 0; i < 6; i++) {
        R[i + 6 * i] = dv6[i];
    }
}

/* EKF更新函数 */
void updateEKFQuatAtt(
    const float gyr_rps[3], const float acc_mps2[3], const float mag_unit[3], float Va_mps, float magDecRad, float T, float NdivT, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float       mnorm;
    int         i0;
    int         iy;
    float       a;
    float       dv0[12];
    float       unnamed_idx_2;
    float       A[49];
    float       s;
    float       A_tmp;
    int         k;
    float       b_a[7];
    float       b_A[49];
    float       dv1[49];
    float       smag;
    float       cmag;
    int         kBcol;
    int         jA;
    float       C[42];
    float       C_tmp;
    float       b_tmp[42];
    int         j;
    int         jj;
    float       K[42];
    int         jp1j;
    signed char ipiv[6];
    int         n;
    float       b_C[42];
    int         ix;
    float       B[36];
    float       dv2[6];
    float       dv3[6];

    /* Get measurements and low-pass filter them */
    p  = lpfGyr * p + (1.0f - lpfGyr) * gyr_rps[0];
    q  = lpfGyr * q + (1.0f - lpfGyr) * gyr_rps[1];
    r  = lpfGyr * r + (1.0f - lpfGyr) * gyr_rps[2];
    ax = lpfAcc * ax + (1.0f - lpfAcc) * acc_mps2[0];
    ay = lpfAcc * ay + (1.0f - lpfAcc) * acc_mps2[1];
    az = lpfAcc * az + (1.0f - lpfAcc) * acc_mps2[2];
    mx = lpfMag * mx + (1.0f - lpfMag) * mag_unit[0];
    my = lpfMag * my + (1.0f - lpfMag) * mag_unit[1];
    mz = lpfMag * mz + (1.0f - lpfMag) * mag_unit[2];

    mnorm = sqrtf((mx * mx + my * my) + mz * mz);
    if (mnorm > 1e-6f) {
        mx /= mnorm;
        my /= mnorm;
        mz /= mnorm;
    }

    Va = lpfVa * Va + (1.0f - lpfVa) * Va_mps;
    i0 = (int) NdivT;
    for (iy = 0; iy < i0; iy++) {
        /* Update state estimate */
        a             = T / NdivT;
        mnorm         = 0.5f * -x[1];
        dv0[0]        = mnorm;
        unnamed_idx_2 = 0.5f * -x[2];
        dv0[4]        = unnamed_idx_2;
        s             = 0.5f * -x[3];
        dv0[8]        = s;
        dv0[1]        = 0.5f * x[0];
        dv0[5]        = s;
        dv0[9]        = 0.5f * x[2];
        dv0[2]        = 0.5f * x[3];
        dv0[6]        = 0.5f * x[0];
        dv0[10]       = mnorm;
        dv0[3]        = unnamed_idx_2;
        dv0[7]        = 0.5f * x[1];
        dv0[11]       = 0.5f * x[0];
        mnorm         = p - x[4];
        s             = q - x[5];
        unnamed_idx_2 = r - x[6];
        for (k = 0; k < 4; k++) {
            b_a[k] = a * ((dv0[k] * mnorm + dv0[k + 4] * s) + dv0[k + 8] * unnamed_idx_2);
        }

        b_a[4] = 0.0f;
        b_a[5] = 0.0f;
        b_a[6] = 0.0f;
        for (k = 0; k < 7; k++) {
            x[k] += b_a[k];
        }
    }

    /* Normalise quaternion */
    mnorm = sqrtf(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
    x[0] /= mnorm;
    x[1] /= mnorm;
    x[2] /= mnorm;
    x[3] /= mnorm;

    /* Compute Jacobian of f, A(x, u) */
    A[0]          = 0.0f;
    mnorm         = p - x[4];
    A[7]          = -0.5f * mnorm;
    unnamed_idx_2 = q - x[5];
    s             = -0.5f * unnamed_idx_2;
    A[14]         = s;
    a             = r - x[6];
    A_tmp         = -0.5f * a;
    A[21]         = A_tmp;
    A[28]         = 0.5f * x[1];
    A[35]         = 0.5f * x[2];
    A[42]         = 0.5f * x[3];
    mnorm *= 0.5f;
    A[1] = mnorm;
    A[8] = 0.0f;
    a *= 0.5f;
    A[15] = a;
    A[22] = s;
    A[29] = -0.5f * x[0];
    A[36] = 0.5f * x[3];
    A[43] = -0.5f * x[2];
    s     = 0.5f * unnamed_idx_2;
    A[2]  = s;
    A[9]  = A_tmp;
    A[16] = 0.0f;
    A[23] = mnorm;
    A[30] = -0.5f * x[3];
    A[37] = -0.5f * x[0];
    A[44] = 0.5f * x[1];
    A[3]  = a;
    A[10] = s;
    A[17] = -0.5f * (p - x[4]);
    A[24] = 0.0f;
    A[31] = 0.5f * x[2];
    A[38] = -0.5f * x[1];
    A[45] = -0.5f * x[0];
    for (i0 = 0; i0 < 7; i0++) {
        A[4 + 7 * i0] = 0.0f;
        A[5 + 7 * i0] = 0.0f;
        A[6 + 7 * i0] = 0.0f;
    }

    /* Update error covariance matrix */
    for (i0 = 0; i0 < 7; i0++) {
        for (k = 0; k < 7; k++) {
            iy            = i0 + 7 * k;
            dv1[iy]       = 0.0f;
            mnorm         = 0.0f;
            unnamed_idx_2 = 0.0f;
            for (kBcol = 0; kBcol < 7; kBcol++) {
                jA = i0 + 7 * kBcol;
                mnorm += A[jA] * P[kBcol + 7 * k];
                unnamed_idx_2 += P[jA] * A[k + 7 * kBcol];
            }
            dv1[iy] = unnamed_idx_2;
            b_A[iy] = mnorm;
        }
    }

    for (i0 = 0; i0 < 49; i0++) {
        P[i0] += T * ((b_A[i0] + dv1[i0]) + Q[i0]);
    }

    smag = sinf(magDecRad);
    cmag = cosf(magDecRad);

    C[0]          = 2.0f * g * x[2];
    mnorm         = -2.0f * g * x[3];
    C[6]          = mnorm;
    C[12]         = 2.0f * g * x[0];
    unnamed_idx_2 = -2.0f * g * x[1];
    C[18]         = unnamed_idx_2;
    C[24]         = 0.0f;
    C[30]         = 0.0f;
    C[36]         = 0.0f;
    C[1]          = unnamed_idx_2;
    C[7]          = -2.0f * g * x[0];
    C[13]         = mnorm;
    C[19]         = -2.0f * g * x[2];
    C[25]         = 0.0f;
    C[31]         = 0.0f;
    C[37]         = -Va;
    C[2]          = 0.0f;
    C[8]          = 4.0f * g * x[1];
    C[14]         = 4.0f * g * x[2];
    C[20]         = 0.0f;
    C[26]         = 0.0f;
    C[32]         = Va;
    C[38]         = 0.0f;
    mnorm         = 2.0f * x[3] * smag;
    C[3]          = mnorm;
    unnamed_idx_2 = 2.0f * x[2] * smag;
    C[9]          = unnamed_idx_2;
    s             = 2.0f * x[1] * smag;
    C[15]         = s - 4.0f * x[2] * cmag;
    A_tmp         = 2.0f * x[0] * smag;
    C[21]         = A_tmp - 4.0f * x[3] * cmag;
    C[27]         = 0.0f;
    C[33]         = 0.0f;
    C[39]         = 0.0f;
    C[4]          = -2.0f * x[3] * cmag;
    a             = 2.0f * x[2] * cmag;
    C[10]         = a - 4.0f * x[1] * smag;
    C_tmp         = 2.0f * x[1] * cmag;
    C[16]         = C_tmp;
    C[22]         = -2.0f * x[0] * cmag - 4.0f * x[3] * smag;
    C[28]         = 0.0f;
    C[34]         = 0.0f;
    C[40]         = 0.0f;
    C[5]          = a - s;
    C[11]         = 2.0f * x[3] * cmag - A_tmp;
    C[17]         = 2.0f * x[0] * cmag + mnorm;
    C[23]         = C_tmp + unnamed_idx_2;
    C[29]         = 0.0f;
    C[35]         = 0.0f;
    C[41]         = 0.0f;

    /* Kalman gain */
    for (i0 = 0; i0 < 6; i0++) {
        for (k = 0; k < 7; k++) {
            b_tmp[k + 7 * i0] = C[i0 + 6 * k];
        }
    }

    for (i0 = 0; i0 < 7; i0++) {
        for (k = 0; k < 6; k++) {
            mnorm = 0.0f;
            for (iy = 0; iy < 7; iy++) {
                mnorm += P[i0 + 7 * iy] * b_tmp[iy + 7 * k];
            }
            K[i0 + 7 * k] = mnorm;
        }
    }

    for (i0 = 0; i0 < 6; i0++) {
        for (k = 0; k < 7; k++) {
            mnorm = 0.0f;
            for (iy = 0; iy < 7; iy++) {
                mnorm += C[i0 + 6 * iy] * P[iy + 7 * k];
            }
            b_C[i0 + 6 * k] = mnorm;
        }
        for (k = 0; k < 6; k++) {
            mnorm = 0.0f;
            for (iy = 0; iy < 7; iy++) {
                mnorm += b_C[i0 + 6 * iy] * b_tmp[iy + 7 * k];
            }
            iy    = i0 + 6 * k;
            B[iy] = mnorm + R[iy];
        }
        ipiv[i0] = (signed char) (1 + i0);
    }

    for (j = 0; j < 5; j++) {
        jA    = j * 7;
        jj    = j * 7;
        jp1j  = jA + 2;
        n     = 6 - j;
        kBcol = 0;
        ix    = jA;
        mnorm = fabsf(B[jA]);
        for (k = 2; k <= n; k++) {
            ix++;
            s = fabsf(B[ix]);
            if (s > mnorm) {
                kBcol = k - 1;
                mnorm = s;
            }
        }

        if (B[jj + kBcol] != 0.0f) {
            if (kBcol != 0) {
                iy      = j + kBcol;
                ipiv[j] = (signed char) (iy + 1);
                ix      = j;
                for (k = 0; k < 6; k++) {
                    mnorm = B[ix];
                    B[ix] = B[iy];
                    B[iy] = mnorm;
                    ix += 6;
                    iy += 6;
                }
            }
            i0 = (jj - j) + 6;
            for (n = jp1j; n <= i0; n++) {
                B[n - 1] /= B[jj];
            }
        }

        n  = 4 - j;
        iy = jA + 6;
        jA = jj;
        for (kBcol = 0; kBcol <= n; kBcol++) {
            mnorm = B[iy];
            if (B[iy] != 0.0f) {
                ix = jj + 1;
                i0 = jA + 8;
                k  = (jA - j) + 12;
                for (jp1j = i0; jp1j <= k; jp1j++) {
                    B[jp1j - 1] += B[ix] * -mnorm;
                    ix++;
                }
            }
            iy += 6;
            jA += 6;
        }
    }

    for (j = 0; j < 6; j++) {
        jA = 7 * j - 1;
        iy = 6 * j;
        for (k = 0; k < j; k++) {
            kBcol = 7 * k;
            mnorm = B[k + iy];
            if (mnorm != 0.0f) {
                for (n = 0; n < 7; n++) {
                    jp1j = (n + jA) + 1;
                    K[jp1j] -= mnorm * K[n + kBcol];
                }
            }
        }
        mnorm = 1.0f / B[j + iy];
        for (n = 0; n < 7; n++) {
            jp1j = (n + jA) + 1;
            K[jp1j] *= mnorm;
        }
    }

    for (j = 5; j >= 0; j--) {
        jA = 7 * j - 1;
        iy = 6 * j - 1;
        i0 = j + 2;
        for (k = i0; k < 7; k++) {
            kBcol = 7 * (k - 1);
            mnorm = B[k + iy];
            if (mnorm != 0.0f) {
                for (n = 0; n < 7; n++) {
                    jp1j = (n + jA) + 1;
                    K[jp1j] -= mnorm * K[n + kBcol];
                }
            }
        }
    }

    for (iy = 4; iy >= 0; iy--) {
        if (ipiv[iy] != iy + 1) {
            for (kBcol = 0; kBcol < 7; kBcol++) {
                jA      = kBcol + 7 * iy;
                mnorm   = K[jA];
                jp1j    = kBcol + 7 * (ipiv[iy] - 1);
                K[jA]   = K[jp1j];
                K[jp1j] = mnorm;
            }
        }
    }

    /* Update error covariance matrix */
    memset(&A[0], 0, 49U * sizeof(float));
    for (k = 0; k < 7; k++) {
        A[k + 7 * k] = 1.0f;
    }

    for (i0 = 0; i0 < 7; i0++) {
        for (k = 0; k < 7; k++) {
            mnorm = 0.0f;
            for (iy = 0; iy < 6; iy++) {
                mnorm += K[i0 + 7 * iy] * C[iy + 6 * k];
            }
            iy      = i0 + 7 * k;
            b_A[iy] = A[iy] - mnorm;
        }

        for (k = 0; k < 7; k++) {
            mnorm = 0.0f;
            for (iy = 0; iy < 7; iy++) {
                mnorm += b_A[i0 + 7 * iy] * P[iy + 7 * k];
            }
            A[i0 + 7 * k] = mnorm;
        }
    }

    memcpy(&P[0], &A[0], 49U * sizeof(float));

    dv2[0]        = ax;
    dv2[1]        = ay;
    dv2[2]        = az;
    dv2[3]        = mx;
    dv2[4]        = my;
    dv2[5]        = mz;
    dv3[0]        = -2.0f * g * (x[1] * x[3] - x[2] * x[0]);
    dv3[1]        = Va * (r - x[6]) - 2.0f * g * (x[2] * x[3] + x[1] * x[0]);
    dv3[2]        = -Va * (q - x[5]) - g * (1.0f - 2.0f * (x[1] * x[1] + x[2] * x[2]));
    mnorm         = 2.0f * x[0] * x[3];
    unnamed_idx_2 = 2.0f * x[1] * x[2];
    s             = 2.0f * x[3] * x[3];
    dv3[3]        = smag * (mnorm + unnamed_idx_2) - cmag * ((2.0f * x[2] * x[2] + s) - 1.0f);
    dv3[4]        = -cmag * (mnorm - unnamed_idx_2) - smag * ((2.0f * x[1] * x[1] + s) - 1.0f);
    dv3[5]        = cmag * (2.0f * x[0] * x[2] + 2.0f * x[1] * x[3]) - smag * (2.0f * x[0] * x[1] - 2.0f * x[2] * x[3]);

    for (i0 = 0; i0 < 6; i0++) {
        dv2[i0] -= dv3[i0];
    }

    for (i0 = 0; i0 < 7; i0++) {
        mnorm = 0.0f;
        for (k = 0; k < 6; k++) {
            mnorm += K[i0 + 7 * k] * dv2[k];
        }
        x[i0] += mnorm;
    }

    /* Normalise quaternion */
    mnorm = sqrtf(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
    x[0] /= mnorm;
    x[1] /= mnorm;
    x[2] /= mnorm;
    x[3] /= mnorm;

    /* Store state estimates */
    mnorm         = x[0] * x[0];
    unnamed_idx_2 = x[1] * x[1];
    s             = x[2] * x[2];
    a             = x[3] * x[3];

    *roll_deg  = atan2f(2.0f * (x[0] * x[1] + x[2] * x[3]), ((mnorm + a) - unnamed_idx_2) - s) * 180.0f / 3.14159265f;
    *pitch_deg = asinf(2.0f * (x[0] * x[2] - x[1] * x[3])) * 180.0f / 3.14159265f;
    *yaw_deg   = atan2f(2.0f * (x[0] * x[3] + x[1] * x[2]), ((mnorm + unnamed_idx_2) - s) - a) * 180.0f / 3.14159265f;
}