#ifdef CALIBRATE
#include "calibration.h"
#else
#include "imu_device.h"
#endif

void setup()
{
#ifdef CALIBRATE
  calibration_run();
#else
  imu_device_init();
#endif
}

void loop()
{
#ifndef CALIBRATE
  imu_device_loop();
#endif
}
