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
  /*
  for (size_t i = 0; i < 3; i++)
  {
    m_gyroFilter[i] = new Filter(LPF, 25, GYRO_SAMPLE_FREQ / 1e3f, 0.4);
    if (m_gyroFilter[i]->get_error_flag() != 0)
      filterInitResult = false;

    m_accelFilter[i] = new Filter(LPF, 25, ACCEL_SAMPLE_FREQ / 1e3f, 0.04);
    if (m_accelFilter[i]->get_error_flag() != 0)
      filterInitResult = false;
  }
  */

  return filterInitResult;
}

void IMU_MPU9150::sampleGyro()
{
  // Sample
  static int16_t g[3];
  getRotation(&g[0], &g[1], &g[2]);

  for (size_t i = 0; i < 3; i++)
  {
    // Save raw sample
    m_lastRaw.gyro[i] = g[i];

    // Apply filtering
    if (m_gyroFilter[i])
      g[i] = m_gyroFilter[i]->do_sample(g[i]);
  }

  // Scaling
  float scaleCoeff = 1.0f / INT16_MAX;
  switch (m_gyroFSD)
  {
  case MPU9150_GYRO_FS_250:
    scaleCoeff *= 250.0f;
    break;
  case MPU9150_GYRO_FS_500:
    scaleCoeff *= 500.0f;
    break;
  case MPU9150_GYRO_FS_1000:
    scaleCoeff *= 1000.0f;
    break;
  case MPU9150_GYRO_FS_2000:
    scaleCoeff *= 2000.0f;
    break;
  }

  for (size_t i = 0; i < 3; i++)
    m_lastFiltered.gyro[i] = g[i] * scaleCoeff;
}

void IMU_MPU9150::sampleAccel()
{
  // Sample
  static int16_t a[3];
  getAcceleration(&a[0], &a[1], &a[2]);

  for (size_t i = 0; i < 3; i++)
  {
    // Save raw sample
    m_lastRaw.accel[i] = a[i];

    // Apply filtering
    if (m_accelFilter[i])
      a[i] = m_accelFilter[i]->do_sample(a[i]);
  }

  /* Scaling */
  float scaleCoeff = 1.0f / INT16_MAX;
  switch (m_accelFSD)
  {
  case MPU9150_ACCEL_FS_2:
    scaleCoeff *= 2.0f;
    break;
  case MPU9150_ACCEL_FS_4:
    scaleCoeff *= 4.0f;
    break;
  case MPU9150_ACCEL_FS_8:
    scaleCoeff *= 8.0f;
    break;
  case MPU9150_ACCEL_FS_16:
    scaleCoeff *= 16.0f;
    break;
  }

  for (size_t i = 0; i < 3; i++)
    m_lastFiltered.accel[i] = a[i] * scaleCoeff;
}

void IMU_MPU9150::sampleMag()
{
  // TODO
}
