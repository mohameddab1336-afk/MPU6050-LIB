/*
 * Madgwick_AHRS.c
 *
 * Implementation of the Madgwick AHRS filter.
 *
 * Algorithm:
 *   1. Convert gyroscope input from deg/s to rad/s.
 *   2. Normalise the accelerometer (and optionally the magnetometer) vector.
 *   3. Compute the objective function f and its Jacobian J that quantify
 *      how far the current quaternion is from the measured field direction.
 *   4. Compute the gradient-descent step:  gradient = J^T * f
 *   5. Integrate:  qdot = 0.5 * q x omega  -  beta * gradient_normalised
 *   6. Integrate qdot to obtain the new quaternion and re-normalise it.
 *
 * Reference:
 *   S. Madgwick, "An efficient orientation filter for inertial and
 *   inertial/magnetic sensor arrays", April 2010.
 */

#include "Madgwick_AHRS.h"
#include <math.h>

/* Conversion constants */
#define DEG_TO_RAD  0.0174533f   /* pi / 180 */
#define RAD_TO_DEG  57.29578f    /* 180 / pi */

/* ------------------------------------------------------------------ */
/* Module-level filter instance                                        */
/* ------------------------------------------------------------------ */
static Madgwick_Filter_t filter;

/* ------------------------------------------------------------------ */
/* Helper: fast inverse square-root                                    */
/* ------------------------------------------------------------------ */
static float invSqrt(float x)
{
    /* Use the standard library for correctness and portability.       */
    return 1.0f / sqrtf(x);
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

void Madgwick_Init(float beta, float sampleFreq)
{
    filter.q0 = 1.0f;
    filter.q1 = 0.0f;
    filter.q2 = 0.0f;
    filter.q3 = 0.0f;
    filter.beta = beta;
    filter.invSampleFreq = 1.0f / sampleFreq;
}

/* ------------------------------------------------------------------ */
/* 6-axis update (accel + gyro, no magnetometer)                      */
/* ------------------------------------------------------------------ */
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* Convert gyroscope from deg/s to rad/s */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    /* Rate of change of quaternion from gyroscope */
    qDot0 = 0.5f * (-filter.q1 * gx - filter.q2 * gy - filter.q3 * gz);
    qDot1 = 0.5f * ( filter.q0 * gx + filter.q2 * gz - filter.q3 * gy);
    qDot2 = 0.5f * ( filter.q0 * gy - filter.q1 * gz + filter.q3 * gx);
    qDot3 = 0.5f * ( filter.q0 * gz + filter.q1 * gy - filter.q2 * gx);

    /* Apply feedback only if accelerometer measurement is valid */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalise accelerometer measurement */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Pre-compute repeated terms */
        _2q0 = 2.0f * filter.q0;
        _2q1 = 2.0f * filter.q1;
        _2q2 = 2.0f * filter.q2;
        _2q3 = 2.0f * filter.q3;
        _4q0 = 4.0f * filter.q0;
        _4q1 = 4.0f * filter.q1;
        _4q2 = 4.0f * filter.q2;
        _8q1 = 8.0f * filter.q1;
        _8q2 = 8.0f * filter.q2;
        q0q0 = filter.q0 * filter.q0;
        q1q1 = filter.q1 * filter.q1;
        q2q2 = filter.q2 * filter.q2;
        q3q3 = filter.q3 * filter.q3;

        /* Gradient descent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter.q1
             - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2
             + _4q1 * az;
        s2 = 4.0f * q0q0 * filter.q2 + _2q0 * ax + _4q2 * q3q3
             - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2
             + _4q2 * az;
        s3 = 4.0f * q1q1 * filter.q3 - _2q1 * ax
             + 4.0f * q2q2 * filter.q3 - _2q2 * ay;

        /* Normalise step magnitude (skip if gradient is zero) */
        float gradSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (gradSq > 0.0f)
        {
            recipNorm = invSqrt(gradSq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* Apply feedback */
            qDot0 -= filter.beta * s0;
            qDot1 -= filter.beta * s1;
            qDot2 -= filter.beta * s2;
            qDot3 -= filter.beta * s3;
        }
    }

    /* Integrate rate of change of quaternion */
    filter.q0 += qDot0 * filter.invSampleFreq;
    filter.q1 += qDot1 * filter.invSampleFreq;
    filter.q2 += qDot2 * filter.invSampleFreq;
    filter.q3 += qDot3 * filter.invSampleFreq;

    /* Normalise quaternion */
    recipNorm = invSqrt(filter.q0 * filter.q0 + filter.q1 * filter.q1
                        + filter.q2 * filter.q2 + filter.q3 * filter.q3);
    filter.q0 *= recipNorm;
    filter.q1 *= recipNorm;
    filter.q2 *= recipNorm;
    filter.q3 *= recipNorm;
}

/* ------------------------------------------------------------------ */
/* 9-axis update (accel + gyro + magnetometer)                        */
/* ------------------------------------------------------------------ */
void Madgwick_Update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz;
    float _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3, q3q3;

    /* Fall back to IMU-only update if magnetometer reading is invalid */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Madgwick_UpdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    /* Convert gyroscope from deg/s to rad/s */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    /* Rate of change of quaternion from gyroscope */
    qDot0 = 0.5f * (-filter.q1 * gx - filter.q2 * gy - filter.q3 * gz);
    qDot1 = 0.5f * ( filter.q0 * gx + filter.q2 * gz - filter.q3 * gy);
    qDot2 = 0.5f * ( filter.q0 * gy - filter.q1 * gz + filter.q3 * gx);
    qDot3 = 0.5f * ( filter.q0 * gz + filter.q1 * gy - filter.q2 * gx);

    /* Apply feedback only if measurements are valid */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalise accelerometer */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Normalise magnetometer */
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* Pre-compute quaternion products */
        q0q0 = filter.q0 * filter.q0;
        q0q1 = filter.q0 * filter.q1;
        q0q2 = filter.q0 * filter.q2;
        q0q3 = filter.q0 * filter.q3;
        q1q1 = filter.q1 * filter.q1;
        q1q2 = filter.q1 * filter.q2;
        q1q3 = filter.q1 * filter.q3;
        q2q2 = filter.q2 * filter.q2;
        q2q3 = filter.q2 * filter.q3;
        q3q3 = filter.q3 * filter.q3;

        /* Reference direction of Earth's magnetic field */
        _2q0mx = 2.0f * filter.q0 * mx;
        _2q0my = 2.0f * filter.q0 * my;
        _2q0mz = 2.0f * filter.q0 * mz;
        _2q1mx = 2.0f * filter.q1 * mx;

        hx = mx * q0q0 - _2q0my * filter.q3 + _2q0mz * filter.q2
             + mx * q1q1 + 2.0f * filter.q1 * my * filter.q2
             + 2.0f * filter.q1 * mz * filter.q3
             - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * filter.q3 + my * q0q0 - _2q0mz * filter.q1
             + _2q1mx * filter.q2 - my * q1q1
             + my * q2q2 + 2.0f * filter.q2 * mz * filter.q3
             - my * q3q3;

        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * filter.q2 + _2q0my * filter.q1
               + mz * q0q0 + _2q1mx * filter.q3
               - mz * q1q1 + 2.0f * filter.q2 * my * filter.q3
               - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Pre-compute more repeated products */
        _2q0 = 2.0f * filter.q0;
        _2q1 = 2.0f * filter.q1;
        _2q2 = 2.0f * filter.q2;
        _2q3 = 2.0f * filter.q3;
        _2q0q2 = 2.0f * filter.q0 * filter.q2;
        _2q2q3 = 2.0f * filter.q2 * filter.q3;

        /* Gradient descent corrective step */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
             + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
             - _2bz * filter.q2
               * (_2bx * (0.5f - q2q2 - q3q3)
                  + _2bz * (q1q3 - q0q2) - mx)
             + (-_2bx * filter.q3 + _2bz * filter.q1)
               * (_2bx * (q1q2 - q0q3)
                  + _2bz * (q0q1 + q2q3) - my)
             + _2bx * filter.q2
               * (_2bx * (q0q2 + q1q3)
                  + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
             + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
             - 4.0f * filter.q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
             + _2bz * filter.q3
               * (_2bx * (0.5f - q2q2 - q3q3)
                  + _2bz * (q1q3 - q0q2) - mx)
             + (_2bx * filter.q2 + _2bz * filter.q0)
               * (_2bx * (q1q2 - q0q3)
                  + _2bz * (q0q1 + q2q3) - my)
             + (_2bx * filter.q3 - _4bz * filter.q1)
               * (_2bx * (q0q2 + q1q3)
                  + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
             + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
             - 4.0f * filter.q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
             + (-_4bx * filter.q2 - _2bz * filter.q0)
               * (_2bx * (0.5f - q2q2 - q3q3)
                  + _2bz * (q1q3 - q0q2) - mx)
             + (_2bx * filter.q1 + _2bz * filter.q3)
               * (_2bx * (q1q2 - q0q3)
                  + _2bz * (q0q1 + q2q3) - my)
             + (_2bx * filter.q0 - _4bz * filter.q2)
               * (_2bx * (q0q2 + q1q3)
                  + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
             + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
             + (-_4bx * filter.q3 + _2bz * filter.q1)
               * (_2bx * (0.5f - q2q2 - q3q3)
                  + _2bz * (q1q3 - q0q2) - mx)
             + (-_2bx * filter.q0 + _2bz * filter.q2)
               * (_2bx * (q1q2 - q0q3)
                  + _2bz * (q0q1 + q2q3) - my)
             + _2bx * filter.q1
               * (_2bx * (q0q2 + q1q3)
                  + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalise step (skip if gradient is zero) */
        float gradSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (gradSq > 0.0f)
        {
            recipNorm = invSqrt(gradSq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* Apply feedback */
            qDot0 -= filter.beta * s0;
            qDot1 -= filter.beta * s1;
            qDot2 -= filter.beta * s2;
            qDot3 -= filter.beta * s3;
        }
    }

    /* Integrate rate of change of quaternion */
    filter.q0 += qDot0 * filter.invSampleFreq;
    filter.q1 += qDot1 * filter.invSampleFreq;
    filter.q2 += qDot2 * filter.invSampleFreq;
    filter.q3 += qDot3 * filter.invSampleFreq;

    /* Normalise quaternion */
    recipNorm = invSqrt(filter.q0 * filter.q0 + filter.q1 * filter.q1
                        + filter.q2 * filter.q2 + filter.q3 * filter.q3);
    filter.q0 *= recipNorm;
    filter.q1 *= recipNorm;
    filter.q2 *= recipNorm;
    filter.q3 *= recipNorm;
}

/* ------------------------------------------------------------------ */
/* Extract Euler angles (degrees) from quaternion                     */
/* ------------------------------------------------------------------ */
void Madgwick_GetEulerAngles(float *roll, float *pitch, float *yaw)
{
    /* Roll  (rotation about X): atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2)) */
    *roll  = RAD_TO_DEG * atan2f(2.0f * (filter.q0 * filter.q1
                                      + filter.q2 * filter.q3),
                              1.0f - 2.0f * (filter.q1 * filter.q1
                                             + filter.q2 * filter.q2));

    /* Pitch (rotation about Y): asin(2*(q0*q2 - q3*q1)) */
    *pitch = RAD_TO_DEG * asinf(2.0f * (filter.q0 * filter.q2
                                     - filter.q3 * filter.q1));

    /* Yaw   (rotation about Z): atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2)) */
    *yaw   = RAD_TO_DEG * atan2f(2.0f * (filter.q0 * filter.q3
                                      + filter.q1 * filter.q2),
                              1.0f - 2.0f * (filter.q2 * filter.q2
                                             + filter.q3 * filter.q3));
}

/* ------------------------------------------------------------------ */
/* Copy current quaternion to caller's variables                      */
/* ------------------------------------------------------------------ */
void Madgwick_GetQuaternion(float *q0, float *q1, float *q2, float *q3)
{
    *q0 = filter.q0;
    *q1 = filter.q1;
    *q2 = filter.q2;
    *q3 = filter.q3;
}
