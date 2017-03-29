#pragma once

#include <Filter.h>

class IMU
{
public:
  struct IMUData
  {
    float gyro[3];
    float acc[3];
    float mag[3];
  };

public:
  IMU()
  {
    for (size_t i = 0; i < 3; i++)
    {
      m_gyroFilter[i] = nullptr;
      m_accelFilter[i] = nullptr;
      m_magFilter[i] = nullptr;
    }
  }

  virtual ~IMU()
  {
  }

  virtual bool init() = 0;
  virtual void sample() = 0;

  const IMUData &rawData() const
  {
    return m_lastRaw;
  }

  const IMUData &filteredData() const
  {
    return m_lastFiltered;
  }

protected:
  IMUData m_lastRaw;
  IMUData m_lastFiltered;

  Filter *m_gyroFilter[3];
  Filter *m_accelFilter[3];
  Filter *m_magFilter[3];
};
