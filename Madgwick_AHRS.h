/*
 * Madgwick AHRS Filter
 * Based on the algorithm by Sebastian Madgwick
 * See: http://www.x-io.co.uk/node/8
 *
 * Provides 6-axis (accelerometer + gyroscope) AHRS filter
 * for computing Roll, Pitch, and Yaw orientation angles.
 */

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <stdint.h>

/* Structure to hold Roll, Pitch, Yaw Euler angles (in degrees) */
typedef struct
{
    float roll;   /* Rotation around X axis, range: -180 to +180 degrees */
    float pitch;  /* Rotation around Y axis, range:  -90 to  +90 degrees */
    float yaw;    /* Rotation around Z axis, range:    0 to  360 degrees */
} EulerAngles_t;

/*
 * @brief  Initialize the Madgwick filter
 * @param  beta:        Filter gain (convergence rate). Recommended 0.1 for drones.
 * @param  sampleFreq:  Sample frequency in Hz (e.g. 200.0f for 200 Hz)
 * @retval None
 */
void Madgwick_Init(float beta, float sampleFreq);

/*
 * @brief  Update the Madgwick filter with new IMU data (6-axis)
 * @param  gx, gy, gz:  Gyroscope measurements in degrees/sec
 * @param  ax, ay, az:  Accelerometer measurements in g
 * @param  dt:          Time step in seconds (e.g. 0.005f for 200 Hz)
 * @retval None
 */
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt);

/*
 * @brief  Get the current orientation as Euler angles
 * @retval EulerAngles_t structure containing roll, pitch, yaw in degrees
 */
EulerAngles_t Madgwick_GetEulerAngles(void);

#endif /* MADGWICK_AHRS_H */
