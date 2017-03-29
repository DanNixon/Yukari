#include "IMU_MPU9150.h"

bool IMU_MPU9150::init()
{
  // Init bus and device
  initialize();

  // Test communication
  if (!testConnection())
    return false;

  // Set ranges
  setFullScaleGyroRange(MPU9150_GYRO_FS_2000);
  setFullScaleAccelRange(MPU9150_ACCEL_FS_16);

  // Cache ranges
  m_gyroFSD = getFullScaleGyroRange();
  m_accelFSD = getFullScaleAccelRange();

  // Init filters
  bool filterInitResult = true;
  for (size_t i = 0; i < 3; i++)
  {
    m_gyroFilter[i] = new Filter(LPF, 20, 4.0, 0.8);
    if (m_gyroFilter[i]->get_error_flag() != 0)
      filterInitResult = false;

    m_accelFilter[i] = new Filter(LPF, 20, 4.0, 0.5);
    if (m_accelFilter[i]->get_error_flag() != 0)
      filterInitResult = false;
  }

  return filterInitResult;
}

void IMU_MPU9150::sample()
{
  // Sample
  static int16_t g[3];
  static int16_t a[3];
  static int16_t m[3];
  getMotion9(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2], &m[0], &m[1], &m[2]);

  // Filtering
  for (size_t i = 0; i < 3; i++)
  {
    if (m_gyroFilter[i])
      g[i] = m_gyroFilter[i]->do_sample(g[i]);

    if (m_accelFilter[i])
      a[i] = m_accelFilter[i]->do_sample(a[i]);

    if (m_magFilter[i])
      m[i] = m_magFilter[i]->do_sample(m[i]);
  }

  // Scaling
  float gyroCoeff = 1.0f / INT16_MAX;
  float accCoeff = 1.0f / INT16_MAX;
  float magCoeff = 1.0f / INT16_MAX;

  switch (m_gyroFSD)
  {
  case MPU9150_GYRO_FS_250:
    gyroCoeff *= 250.0f;
    break;
  case MPU9150_GYRO_FS_500:
    gyroCoeff *= 500.0f;
    break;
  case MPU9150_GYRO_FS_1000:
    gyroCoeff *= 1000.0f;
    break;
  case MPU9150_GYRO_FS_2000:
    gyroCoeff *= 2000.0f;
    break;
  }

  switch (m_accelFSD)
  {
  case MPU9150_ACCEL_FS_2:
    accCoeff *= 2.0f;
    break;
  case MPU9150_ACCEL_FS_4:
    accCoeff *= 4.0f;
    break;
  case MPU9150_ACCEL_FS_8:
    accCoeff *= 8.0f;
    break;
  case MPU9150_ACCEL_FS_16:
    accCoeff *= 16.0f;
    break;
  }

  for (size_t i = 0; i < 3; i++)
  {
    m_lastSample.gyro[i] = g[i] * gyroCoeff;
    m_lastSample.acc[i] = a[i] * accCoeff;
    m_lastSample.mag[i] = m[i] * magCoeff;
  }
}
