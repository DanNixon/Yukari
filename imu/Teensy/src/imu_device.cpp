#include "imu_device.h"

#include <I2Cdev.h>

#include <BMP085.h>
#include <MPU9150_9Axis_MotionApps41.h>
#include <TinyGPS++.h>
#include <helper_3dmath.h>

#include <MSP.h>
#include <Scheduler.h>

Scheduler g_scheduler;

MSP g_msp(MSP_SERIAL);

TinyGPSPlus g_gps;

BMP085 g_barometer;
float g_temperature;
float g_pressure;
float g_altitude;
int32_t g_baroLastMicros = 0;

MPU9150 g_imu;
uint16_t g_dmpFIFOPacketSize;
uint16_t g_dmpFIFOBufferSize;
uint8_t g_dmpFIFOBuffer[64];
volatile bool g_mpuInterrupt = false;

// DMP packets use 1g = 4096 (AFS_SEL=2)
const float ACCEL_COEFF = 9.81f / 4096.0f;

Quaternion g_quat;
VectorFloat g_gravity;
VectorInt16 g_accelCalib;
VectorInt16 g_accel;
VectorInt16 g_realAccel;
VectorInt16 g_worldAccel;
VectorFloat g_worldAccelMS2;

uint8_t g_accelSamples = 0;
VectorFloat g_worldAccelMS2LPFAccum;
VectorFloat g_worldAccelMS2LPF;

uint32_t g_lastIntegrationTimestep = 0;
VectorFloat g_velocity;
VectorFloat g_displacement;

void dmpDataReady()
{
  g_mpuInterrupt = true;
}

void taskBlink()
{
  static bool blinkState = false;
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void taskDMP()
{
  if (!g_mpuInterrupt && g_dmpFIFOBufferSize < g_dmpFIFOPacketSize)
    return;

  // reset interrupt flag and get INT_STATUS byte
  g_mpuInterrupt = false;
  uint8_t mpuIntStatus = g_imu.getIntStatus();

  // get current FIFO count
  g_dmpFIFOBufferSize = g_imu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || g_dmpFIFOBufferSize == 1024)
  {
    // reset so we can continue cleanly
    g_imu.resetFIFO();
#ifdef DEBUG
    DEBUG_SERIAL.printf("FIFO overflow!\n");
#endif
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (g_dmpFIFOBufferSize < g_dmpFIFOPacketSize)
      g_dmpFIFOBufferSize = g_imu.getFIFOCount();

    // read a packet from FIFO
    g_imu.getFIFOBytes(g_dmpFIFOBuffer, g_dmpFIFOPacketSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    g_dmpFIFOBufferSize -= g_dmpFIFOPacketSize;

    g_imu.dmpGetQuaternion(&g_quat, g_dmpFIFOBuffer);
    g_imu.dmpGetAccel(&g_accel, g_dmpFIFOBuffer);
    g_accel.x -= g_accelCalib.x;
    g_accel.y -= g_accelCalib.y;
    g_accel.z -= g_accelCalib.z;
    g_imu.dmpGetGravity(&g_gravity, &g_quat);
    g_imu.dmpGetLinearAccel(&g_realAccel, &g_accel, &g_gravity);
    g_imu.dmpGetLinearAccelInWorld(&g_worldAccel, &g_realAccel, &g_quat);

    g_worldAccelMS2 =
        VectorFloat(g_worldAccel.x, g_worldAccel.y, g_worldAccel.z - 5150) * ACCEL_COEFF;
    g_worldAccelMS2LPFAccum += g_worldAccelMS2 / 8.0f;
    g_accelSamples++;

    // Calculate velocity and displacement
    uint32_t now = micros();
    if (g_lastIntegrationTimestep != 0 && g_accelSamples == 8)
    {
      g_worldAccelMS2LPF = g_worldAccelMS2LPFAccum;
      g_worldAccelMS2LPFAccum.toZero();
      g_accelSamples = 0;

      float deltaTime = (float)(now - g_lastIntegrationTimestep) * 1e-6;
      g_velocity += g_worldAccelMS2LPF * deltaTime;
      g_displacement += g_velocity * deltaTime;
    }

    g_lastIntegrationTimestep = now;
  }
}

void taskResetIMUIntegration()
{
  if (digitalRead(10) == LOW)
  {
    g_velocity.toZero();
    g_displacement.toZero();

#ifdef DEBUG
    DEBUG_SERIAL.printf("IMU integration reset\n");
#endif
  }
}

void taskBarometer()
{
  g_baroLastMicros = micros();

  g_barometer.setControl(BMP085_MODE_TEMPERATURE);
  while (micros() - g_baroLastMicros < g_barometer.getMeasureDelayMicroseconds())
    ;

  g_temperature = g_barometer.getTemperatureC();

  g_barometer.setControl(BMP085_MODE_PRESSURE_3);
  while (micros() - g_baroLastMicros < g_barometer.getMeasureDelayMicroseconds())
    ;

  g_pressure = g_barometer.getPressure();
  g_altitude = g_barometer.getAltitude(g_pressure);
}

#ifdef DEBUG
void taskPrintData()
{
  // Device orientation as quaternion
  DEBUG_SERIAL.printf("quat\t");
  DEBUG_SERIAL.print(g_quat.w);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_quat.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_quat.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_quat.z);

  // Linear acceleration
  DEBUG_SERIAL.printf("a\t%d\t%d\t%d\n", g_accel.x, g_accel.y, g_accel.z);

  // Linear acceleration without gravity
  DEBUG_SERIAL.printf("aReal\t%d\t%d\t%d\n", g_realAccel.x, g_realAccel.y, g_realAccel.z);

  // Linear acceleration without gravity and corrected for orientation
  DEBUG_SERIAL.printf("aWorld\t%d\t%d\t%d\n", g_worldAccel.x, g_worldAccel.y, g_worldAccel.z);

  // Linear acceleration without gravity and corrected for orientation in ms-2
  DEBUG_SERIAL.print("aWorld (ms-2)\t");
  DEBUG_SERIAL.print(g_worldAccelMS2.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_worldAccelMS2.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_worldAccelMS2.z);

  // Linear acceleration without gravity and corrected for orientation in ms-2 with low pass filter
  DEBUG_SERIAL.print("aWorld LPF (ms-2)\t");
  DEBUG_SERIAL.print(g_worldAccelMS2LPF.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_worldAccelMS2LPF.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_worldAccelMS2LPF.z);

  // Velocity
  DEBUG_SERIAL.printf("velocity (ms-1)\t");
  DEBUG_SERIAL.print(g_velocity.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_velocity.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_velocity.z);

  // Displacement
  DEBUG_SERIAL.printf("displacement (m)\t");
  DEBUG_SERIAL.print(g_displacement.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_displacement.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_displacement.z);

  // BMP180 data
  DEBUG_SERIAL.printf("T/P/Alt\t");
  DEBUG_SERIAL.print(g_temperature);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_pressure);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_altitude);
}
#endif /* DEBUG */

#ifdef SERIALPLOT
/*
 * Prints debug data as CSV for serialplot.
 *
 *  1) Quaternion - w
 *  2) Quaternion - x
 *  3) Quaternion - y
 *  4) Quaternion - z
 *  5) Acceleration - x
 *  6) Acceleration - y
 *  7) Acceleration - z
 *  8) Linear acceleration - x
 *  9) Linear acceleration - y
 * 10) Linear acceleration - z
 * 11) World acceleration - x
 * 12) World acceleration - y
 * 13) World acceleration - z
 * 14) Velocity - x
 * 15) Velocity - y
 * 16) Velocity - z
 * 17) Displacement - x
 * 18) Displacement - y
 * 19) Displacement - z
 */
void taskDebugPrintCSV()
{
  DEBUG_SERIAL.print(g_quat.w);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_quat.x);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_quat.y);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_quat.z);
  DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,", g_accel.x, g_accel.y, g_accel.z, g_realAccel.x,
                      g_realAccel.y, g_realAccel.z, g_worldAccel.x, g_worldAccel.y, g_worldAccel.z);

  DEBUG_SERIAL.print(g_velocity.x);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_velocity.y);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_velocity.z);
  DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(g_displacement.x);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_displacement.y);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(g_displacement.z);

  DEBUG_SERIAL.println();
}
#endif /* SERIALPLOT */

void taskFeedGPS()
{
  if (GPS_SERIAL.available())
    g_gps.encode(GPS_SERIAL.read());
}

#if defined(DEBUG)
void taskDebugPrintGPS()
{
  DEBUG_SERIAL.printf("GPS stats\n");

  if (g_gps.satellites.isUpdated())
    DEBUG_SERIAL.printf("Sats: %ld\n", g_gps.satellites.value());

  if (g_gps.location.isUpdated())
  {
    DEBUG_SERIAL.printf("Lat: %c%d.%ld\n", g_gps.location.rawLat().negative ? '-' : '+',
                        g_gps.location.rawLat().deg, g_gps.location.rawLat().billionths);
    DEBUG_SERIAL.printf("Lng: %c%d.%ld\n", g_gps.location.rawLng().negative ? '-' : '+',
                        g_gps.location.rawLng().deg, g_gps.location.rawLng().billionths);
    DEBUG_SERIAL.printf("Age: %ld\n", g_gps.location.age());
  }

  if (g_gps.altitude.isUpdated())
    DEBUG_SERIAL.printf("Alt: %ldcm\n", g_gps.altitude.value());

  if (g_gps.course.isUpdated())
    DEBUG_SERIAL.printf("Course: %ld(deg/100)\n", g_gps.course.value());

  if (g_gps.speed.isUpdated())
    DEBUG_SERIAL.printf("Speed: %ld(knot/100)\n", g_gps.speed.value());
}
#endif /* DEBUG */

void taskMSP()
{
  g_msp.loop();
}

void imu_device_init()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

#if defined(DEBUG) || defined(SERIALPLOT)
  /* Init debug serial */
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL)
    delay(5);
  DEBUG_SERIAL.printf("Serial up\n");
#endif /* DEBUG || SERIALPLOT */

  /* Init MSP */
  MSP_SERIAL.begin(MSP_BAUD);
  g_msp.setOnMessage([](MSP::Direction dir, MSP::Command cmd, uint8_t *buff, uint8_t len) {
#ifdef DEBUG
    DEBUG_SERIAL.printf("dir=");
    DEBUG_SERIAL.println((uint8_t)dir, HEX);
    DEBUG_SERIAL.printf("cmd=");
    DEBUG_SERIAL.println((uint8_t)cmd, HEX);
    DEBUG_SERIAL.printf("len=%d\n", len);
    for (uint8_t i = 0; i < len; i++)
    {
      DEBUG_SERIAL.print(buff[i], HEX);
      DEBUG_SERIAL.print(' ');
    }
    DEBUG_SERIAL.printf("\n");
#endif

    switch (cmd)
    {
    case MSP::Command::Y_ORIENTATION:
    {
      uint8_t pkt[8];
      pkt[0] = g_dmpFIFOBuffer[0];
      pkt[1] = g_dmpFIFOBuffer[1];
      pkt[2] = g_dmpFIFOBuffer[4];
      pkt[3] = g_dmpFIFOBuffer[5];
      pkt[4] = g_dmpFIFOBuffer[8];
      pkt[5] = g_dmpFIFOBuffer[9];
      pkt[6] = g_dmpFIFOBuffer[12];
      pkt[7] = g_dmpFIFOBuffer[13];
      g_msp.sendPacket(MSP::Command::Y_RAW_IMU, pkt, 8);
      break;
    }
    case MSP::Command::Y_DISPLACEMENT:
    {
      /* TODO */
      break;
    }
    case MSP::Command::Y_RESET_DISPLACEMENT:
    {
      /* TODO */
      break;
    }
    default:
      break;
    }
  });
#ifdef DEBUG
  DEBUG_SERIAL.printf("MSP init\n");
#endif /* DEBUG */

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  /* Init i2c bus (Arduino) */
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  /* Init i2c bus (Fast Wire) */
  Fastwire::setup(400, true);
#endif /* I2CDEV_IMPLEMENTATION */
#ifdef DEBUG
  DEBUG_SERIAL.printf("Wire init\n");
#endif /* DEBUG */

#ifndef DISABLE_GPS
  GPS_SERIAL.begin(GPS_BAUD);
#endif /* GPS_SERIAL */
#ifdef DEBUG
  DEBUG_SERIAL.printf("GPS init\n");
  DEBUG_SERIAL.printf("TinyGPS++ version: %s\n", TinyGPSPlus::libraryVersion());
#endif /* DEBUG */

#ifndef DISABLE_IMU
  /* Init IMU */
  g_imu.initialize();
#ifdef DEBU2
  DEBUG_SERIuAL.printf("IMU init: %d\n", g_imu.testConnection());
#endif /* DEBUG */

  /* Calibration sample */
  int64_t axa = 0;
  int64_t aya = 0;
  int64_t aza = 0;
  const uint16_t numSamples = 1000;
  int16_t dummy, ax, ay, az;
#ifdef DEBU2
  DEBUG_SERIuAL.printf("Accel. calibration start\n");
#endif /* DEBUG */
  for (uint16_t i = 0; i < numSamples; i++)
  {
    g_imu.getMotion6(&ax, &ay, &az, &dummy, &dummy, &dummy);
    axa += ax;
    aya += ay;
    aza += az;
    delay(1);
  }
  g_accelCalib.x = axa / numSamples;
  g_accelCalib.y = aya / numSamples;
  g_accelCalib.z = aza / numSamples;
#ifdef DEBU2
  DEBUG_SERIuAL.printf("Accel. calibration done: %d, %d, %d\n", g_accelCalib.x, g_accelCalib.y,
                       g_accelCalib.z);
#endif /* DEBUG */

  pinMode(MPU_INTERRUPT_PIN, INPUT);

  /* Init DMP */
  uint8_t dmpStatus = g_imu.dmpInitialize();
  if (dmpStatus == 0)
  {
    g_imu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    dmpStatus = g_imu.getIntStatus();

    g_dmpFIFOPacketSize = g_imu.dmpGetFIFOPacketSize();
  }
#endif /* DISABLE_IMU */

#ifndef DISABLE_BARO
  /* Init barometer */
  g_barometer.initialize();
#ifdef DEBUG
  DEBUG_SERIAL.printf("Barometer init: %d\n", g_barometer.testConnection());
#endif /* DEBUG */
#endif /* DISABLE_BARO */

  /* Init scheduler */
  g_scheduler.addTask(&taskBlink, Scheduler::HzToUsInterval(5.0f));
  g_scheduler.addTask(&taskMSP, 0);
  g_scheduler.addTask(&taskResetIMUIntegration, Scheduler::HzToUsInterval(10.0f));
#ifdef SERIALPLOT
  g_scheduler.addTask(&taskDebugPrintCSV, Scheduler::HzToUsInterval(10.0f));
#endif /* SERIALPLOT */
#ifndef DISABLE_GPS
  g_scheduler.addTask(&taskFeedGPS, 0);
#endif /* DISABLE_GPS */
#ifndef DISABLE_IMU
  if (dmpStatus == 0)
    g_scheduler.addTask(&taskDMP, 0);
#endif /* DISABLE_IMU */
#ifndef DISABLE_BARO
  g_scheduler.addTask(&taskBarometer, Scheduler::HzToUsInterval(10.0f));
#endif /* DISABLE_BARO */
#ifdef DEBUG
  g_scheduler.addTask(&taskPrintData, Scheduler::HzToUsInterval(10.0f));
#ifndef DISABLE_GPS
  g_scheduler.addTask(&taskDebugPrintGPS, Scheduler::HzToUsInterval(1.0f));
#endif /* DISABLE_GPS */
  g_scheduler.print(DEBUG_SERIAL);
#endif /* DEBUG */

  digitalWrite(LED_PIN, LOW);
#ifdef DEBUG
  DEBUG_SERIAL.printf("Ready\n");
#endif /* DEBUG */
}

void imu_device_loop()
{
  g_scheduler.loop();
}
