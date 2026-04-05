/*
 * Madgwick_AHRS.h
 *
 * Madgwick AHRS (Attitude and Heading Reference System) Filter
 *
 * Fuses gyroscope and accelerometer data (and optionally magnetometer)
 * to produce stable Roll, Pitch, and Yaw angles via quaternion integration.
 *
 * Reference:
 *   S. Madgwick, "An efficient orientation filter for inertial and
 *   inertial/magnetic sensor arrays", April 2010.
 *   https://x-io.co.uk/open-source-ahrs-with-x-imu/
 *
 * Usage example (drone main loop):
 *
 *   #include "TJ_MPU6050.h"
 *   #include "Madgwick_AHRS.h"
 *
 *   // Initialise once (beta = 0.1, sample frequency = 200 Hz)
 *   Madgwick_Init(0.1f, 200.0f);
 *
 *   // Inside the control loop (call at sample frequency)
 *   ScaledData_Def accel, gyro;
 *   MPU6050_Get_Accel_Scale(&accel);
 *   MPU6050_Get_Gyro_Scale(&gyro);
 *
 *   Madgwick_UpdateIMU(gyro.x, gyro.y, gyro.z,
 *                      accel.x, accel.y, accel.z);
 *
 *   float roll, pitch, yaw;
 *   Madgwick_GetEulerAngles(&roll, &pitch, &yaw);
 *
 * Beta tuning guide:
 *   - Lower beta (e.g. 0.01): slower convergence, better noise rejection
 *   - Higher beta (e.g. 0.1 ): faster convergence, less stable
 *   - Recommended for drones: 0.05 – 0.1
 */

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------
 * Filter structure
 * ------------------------------------------------------------------ */
typedef struct
{
    float q0;             /* Quaternion component w   */
    float q1;             /* Quaternion component x   */
    float q2;             /* Quaternion component y   */
    float q3;             /* Quaternion component z   */
    float beta;           /* Filter gain (convergence rate) */
    float invSampleFreq;  /* 1 / sample_frequency_Hz  */
} Madgwick_Filter_t;

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

/**
 * @brief  Initialise the Madgwick filter.
 * @param  beta        Filter gain (typical range 0.01 – 0.15).
 * @param  sampleFreq  Sensor sample frequency in Hz (e.g. 200.0f).
 */
void Madgwick_Init(float beta, float sampleFreq);

/**
 * @brief  Update the filter using 6-axis IMU data (no magnetometer).
 * @param  gx  Gyroscope x in deg/s
 * @param  gy  Gyroscope y in deg/s
 * @param  gz  Gyroscope z in deg/s
 * @param  ax  Accelerometer x (any consistent unit, e.g. mg or g)
 * @param  ay  Accelerometer y
 * @param  az  Accelerometer z
 */
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az);

/**
 * @brief  Update the filter using 9-axis IMU data (with magnetometer).
 * @param  gx  Gyroscope x in deg/s
 * @param  gy  Gyroscope y in deg/s
 * @param  gz  Gyroscope z in deg/s
 * @param  ax  Accelerometer x
 * @param  ay  Accelerometer y
 * @param  az  Accelerometer z
 * @param  mx  Magnetometer x (any consistent unit)
 * @param  my  Magnetometer y
 * @param  mz  Magnetometer z
 */
void Madgwick_Update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz);

/**
 * @brief  Extract Euler angles from the current quaternion.
 * @param  roll   Output: rotation about X axis in degrees [-180, +180]
 * @param  pitch  Output: rotation about Y axis in degrees [ -90,  +90]
 * @param  yaw    Output: rotation about Z axis in degrees [-180, +180]
 */
void Madgwick_GetEulerAngles(float *roll, float *pitch, float *yaw);

/**
 * @brief  Copy the current quaternion to the caller's variables.
 * @param  q0  Output: w component
 * @param  q1  Output: x component
 * @param  q2  Output: y component
 * @param  q3  Output: z component
 */
void Madgwick_GetQuaternion(float *q0, float *q1, float *q2, float *q3);

#ifdef __cplusplus
}
#endif

#endif /* MADGWICK_AHRS_H */
