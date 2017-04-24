#include <I2Cdev.h>

#include <BMP085.h>
#include <MPU9150_9Axis_MotionApps41.h>
#include <TinyGPS++.h>
#include <helper_3dmath.h>

#include <MSP.h>
#include <Scheduler.h>

#define LED_PIN 13
#define MPU_INTERRUPT_PIN 20

Scheduler g_scheduler;

MSP g_msp(MSP_SERIAL);

#ifndef DISABLE_GPS
TinyGPSPlus g_gps;
#endif /* DISABLE_GPS */

#ifndef DISABLE_BARO
BMP085 g_barometer;
float g_temperature;
float g_pressure;
float g_altitude;
int32_t g_baroLastMicros = 0;
#endif /* DISABLE_BARO */

#ifndef DISABLE_IMU
MPU9150 g_imu;
uint16_t g_dmpFIFOPacketSize;
uint16_t g_dmpFIFOBufferSize;
uint8_t g_dmpFIFOBuffer[64];
volatile bool g_mpuInterrupt = false;

Quaternion g_quat;
VectorFloat m_gravity;
VectorInt16 m_accel;
VectorInt16 m_realAccel;
VectorInt16 m_worldAccel;
#endif /* DISABLE_IMU */

#ifdef TEAPOT
uint8_t g_teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
#endif /* TEAPOT */

#ifndef DISABLE_IMU
void dmpDataReady()
{
  g_mpuInterrupt = true;
}
#endif /* DISABLE_IMU */

void taskBlink()
{
  static bool blinkState = false;
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

#ifndef DISABLE_IMU
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
    g_imu.dmpGetAccel(&m_accel, g_dmpFIFOBuffer);
    g_imu.dmpGetGravity(&m_gravity, &g_quat);
    g_imu.dmpGetLinearAccel(&m_realAccel, &m_accel, &m_gravity);
    g_imu.dmpGetLinearAccelInWorld(&m_worldAccel, &m_realAccel, &g_quat);
  }
}
#endif /* DISABLE_IMU */

#ifndef DISABLE_BARO
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
#endif /* DISABLE_BARO */

#ifdef DEBUG
void taskPrintData()
{
#ifndef DISABLE_IMU
  // Device orientation as quaternion
  DEBUG_SERIAL.printf("quat\t%d\t%d\t%d\t%d\n", g_quat.w, g_quat.x, g_quat.z, g_quat.w);

  // Linear acceleration
  DEBUG_SERIAL.printf("a\t%d\t%d\t%d\n", m_accel.x, m_accel.y, m_accel.z);

  // Linear acceleration without gravity
  DEBUG_SERIAL.printf("aReal\t%d\t%d\t%d\n", m_realAccel.x, m_realAccel.y, m_realAccel.z);

  // Linear acceleration without gravity and corrected for orientation
  DEBUG_SERIAL.printf("aWorld\t%d\t%d\t%d\n", m_worldAccel.x, m_worldAccel.y, m_worldAccel.z);
#endif /* DISABLE_IMU */

#ifndef DISABLE_BARO
  // BMP180 data
  DEBUG_SERIAL.printf("T/P/Alt\t");
  DEBUG_SERIAL.print(g_temperature);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_pressure);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_altitude);
#endif /* DISABLE_BARO */
}
#endif /* DEBUG */

#ifdef TEAPOT
void taskTeapot()
{
  g_teapotPacket[2] = g_dmpFIFOBuffer[0];
  g_teapotPacket[3] = g_dmpFIFOBuffer[1];
  g_teapotPacket[4] = g_dmpFIFOBuffer[4];
  g_teapotPacket[5] = g_dmpFIFOBuffer[5];
  g_teapotPacket[6] = g_dmpFIFOBuffer[8];
  g_teapotPacket[7] = g_dmpFIFOBuffer[9];
  g_teapotPacket[8] = g_dmpFIFOBuffer[12];
  g_teapotPacket[9] = g_dmpFIFOBuffer[13];

  DEBUG_SERIAL.write(g_teapotPacket, 14);
}
#endif /* TEAPOT */

#ifndef DISABLE_GPS
void taskFeedGPS()
{
  if (GPS_SERIAL.available())
    g_gps.encode(GPS_SERIAL.read());
}
#endif /* DISABLE_GPS */

#ifdef DEBUG
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

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

#if defined(DEBUG) || defined(TEAPOT)
  /* Init debug serial */
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL)
    delay(5);
  DEBUG_SERIAL.printf("Serial up\n");
#endif /* DEBUG || TEAPOT */

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
    case MSP::Command::Y_RAW_IMU:
    {
      /* TODO */
      break;
    }
    case MSP::Command::Y_LINEAR_ACC_REAL:
    {
      /* TODO */
      break;
    }
    case MSP::Command::Y_LINEAR_ACC_WORLD:
    {
      /* TODO */
      break;
    }
    case MSP::Command::Y_ORIENTATION:
    {
      uint8_t pkt[8];
#ifndef DISABLE_IMU
      pkt[0] = g_dmpFIFOBuffer[0];
      pkt[1] = g_dmpFIFOBuffer[1];
      pkt[2] = g_dmpFIFOBuffer[4];
      pkt[3] = g_dmpFIFOBuffer[5];
      pkt[4] = g_dmpFIFOBuffer[8];
      pkt[5] = g_dmpFIFOBuffer[9];
      pkt[6] = g_dmpFIFOBuffer[12];
      pkt[7] = g_dmpFIFOBuffer[13];
#endif /* DISABLE_IMU */
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

#ifdef GPS_SERIAL
  GPS_SERIAL.begin(GPS_BAUD);
#endif /* GPS_SERIAL */
#ifdef DEBUG
  DEBUG_SERIAL.printf("GPS init\n");
  DEBUG_SERIAL.printf("TinyGPS++ version: %s\n", TinyGPSPlus::libraryVersion());
#endif /* DEBUG */

#ifndef DISABLE_IMU
  /* Init IMU */
  g_imu.initialize();
#ifdef DEBUG
  DEBUG_SERIAL.printf("IMU init: %d\n", g_imu.testConnection());
#endif /* DEBUG */

  g_imu.setFullScaleAccelRange(MPU9150_ACCEL_FS_16);
  g_imu.setFullScaleGyroRange(MPU9150_GYRO_FS_2000);

  pinMode(MPU_INTERRUPT_PIN, INPUT);

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
#ifdef TEAPOT
  g_scheduler.addTask(&taskTeapot, Scheduler::HzToUsInterval(50.0f));
#endif
  g_scheduler.addTask(&taskMSP, 0);
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
  g_scheduler.addTask(&taskDebugPrintGPS, Scheduler::HzToUsInterval(1.0f));
  g_scheduler.print(DEBUG_SERIAL);
#endif /* DEBUG */

  digitalWrite(LED_PIN, LOW);
#ifdef DEBUG
  DEBUG_SERIAL.printf("Ready\n");
#endif /* DEBUG */
}

void loop()
{
  g_scheduler.loop();
}
