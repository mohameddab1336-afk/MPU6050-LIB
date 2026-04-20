/*
 * Madgwick AHRS Filter
 * Based on the algorithm by Sebastian Madgwick
 * See: http://www.x-io.co.uk/node/8
 *
 * Provides 6-axis (accelerometer + gyroscope) AHRS filter
 * for computing Roll, Pitch, and Yaw orientation angles.
 */

#include "Madgwick_AHRS.h"
#include <math.h>

#define PI 3.14159265358979323846f

/* Filter parameters */
static float beta_gain    = 0.1f;   /* Filter gain (convergence rate) */
static float inv_freq     = 0.005f; /* 1 / sampleFreq (seconds per sample) */

/* Quaternion state: represents current orientation */
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

/*
 * @brief  Initialize the Madgwick filter
 * @param  beta:        Filter gain. Typical value: 0.1 for drones.
 * @param  sampleFreq:  Sample frequency in Hz (e.g. 200.0f)
 * @retval None
 */
void Madgwick_Init(float beta, float sampleFreq)
{
    beta_gain = beta;
    if (sampleFreq > 0.0f)
    {
        inv_freq = 1.0f / sampleFreq;
    }
    /* Reset quaternion to identity (no rotation) */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

/*
 * @brief  Update the Madgwick filter with new IMU data (6-axis)
 * @param  gx, gy, gz:  Gyroscope measurements in degrees/sec
 * @param  ax, ay, az:  Accelerometer measurements in g
 * @param  dt:          Time step in seconds
 * @retval None
 */
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float v2q0, v2q1, v2q2, v2q3;
    float v4q0, v4q1, v4q2;
    float v8q1, v8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* Convert gyroscope degrees/sec to radians/sec */
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* Compute feedback only if accelerometer measurement is valid */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalise accelerometer measurement */
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        v2q0 = 2.0f * q0;
        v2q1 = 2.0f * q1;
        v2q2 = 2.0f * q2;
        v2q3 = 2.0f * q3;
        v4q0 = 4.0f * q0;
        v4q1 = 4.0f * q1;
        v4q2 = 4.0f * q2;
        v8q1 = 8.0f * q1;
        v8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        /* Gradient descent algorithm corrective step */
        s0 = v4q0 * q2q2 + v2q2 * ax + v4q0 * q1q1 - v2q1 * ay;
        s1 = v4q1 * q3q3 - v2q3 * ax + 4.0f * q0q0 * q1 - v2q0 * ay - v4q1
             + v8q1 * q1q1 + v8q1 * q2q2 + v4q1 * az;
        s2 = 4.0f * q0q0 * q2 + v2q0 * ax + v4q2 * q3q3 - v2q3 * ay - v4q2
             + v8q2 * q1q1 + v8q2 * q2q2 + v4q2 * az;
        s3 = 4.0f * q1q1 * q3 - v2q1 * ax + 4.0f * q2q2 * q3 - v2q2 * ay;

        /* Normalise step magnitude */
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= beta_gain * s0;
        qDot2 -= beta_gain * s1;
        qDot3 -= beta_gain * s2;
        qDot4 -= beta_gain * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normalise quaternion */
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/*
 * @brief  Get the current orientation as Euler angles computed from
 *         the internal quaternion state.
 * @retval EulerAngles_t structure containing roll, pitch, yaw in degrees
 *         - roll:  rotation around X axis (-180 to +180 degrees)
 *         - pitch: rotation around Y axis ( -90 to  +90 degrees)
 *         - yaw:   rotation around Z axis (   0 to  360 degrees)
 */
EulerAngles_t Madgwick_GetEulerAngles(void)
{
    EulerAngles_t angles;

    /* Roll (rotation around X axis) */
    angles.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / PI);

    /* Pitch (rotation around Y axis) — clamp to avoid asinf domain errors */
    float sinPitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sinPitch >= 1.0f)
    {
        angles.pitch = 90.0f;
    }
    else if (sinPitch <= -1.0f)
    {
        angles.pitch = -90.0f;
    }
    else
    {
        angles.pitch = asinf(sinPitch) * (180.0f / PI);
    }

    /* Yaw (rotation around Z axis) */
    angles.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / PI);

    /* Normalise yaw to [0, 360) degrees */
    if (angles.yaw < 0.0f)
    {
        angles.yaw += 360.0f;
    }

    return angles;
}
