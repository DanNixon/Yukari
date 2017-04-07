#include <I2Cdev.h>

#include <BMP085.h>
#include <MPU9150_9Axis_MotionApps41.h>
#include <helper_3dmath.h>

#include <MSP.h>
#include <Scheduler.h>

#define LED_PIN 13
#define MPU_INTERRUPT_PIN 20

Scheduler g_scheduler;

MSP g_msp(MSP_SERIAL);

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

Quaternion g_quat;
VectorFloat m_gravity;
VectorInt16 m_accel;
VectorInt16 m_realAccel;
VectorInt16 m_worldAccel;

#ifdef TEAPOT
uint8_t g_teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
#endif /* TEAPOT */

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
    DEBUG_SERIAL.println(F("FIFO overflow!"));
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
  DEBUG_SERIAL.print("quat\t");
  DEBUG_SERIAL.print(g_quat.w);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_quat.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_quat.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_quat.z);

  // Linear acceleration
  DEBUG_SERIAL.print("a\t");
  DEBUG_SERIAL.print(m_accel.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(m_accel.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(m_accel.z);

  // Linear acceleration without gravity
  DEBUG_SERIAL.print("areal\t");
  DEBUG_SERIAL.print(m_realAccel.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(m_realAccel.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(m_realAccel.z);

  // Linear acceleration without gravity and corrected for orientation
  DEBUG_SERIAL.print("aworld\t");
  DEBUG_SERIAL.print(m_worldAccel.x);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(m_worldAccel.y);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(m_worldAccel.z);

  // BMP180 data
  DEBUG_SERIAL.print("T/P/Alt\t");
  DEBUG_SERIAL.print(g_temperature);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(g_pressure);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(g_altitude);
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
#endif /* DEBUG || TEAPOT */

  /* Init MSP */
  MSP_SERIAL.begin(MSP_BAUD);
  g_msp.setOnMessage([](MSP::Direction dir, MSP::Command cmd, uint8_t *buff, uint8_t len) {
#ifdef DEBUG
    DEBUG_SERIAL.print("dir=");
    DEBUG_SERIAL.println((uint8_t)dir, HEX);
    DEBUG_SERIAL.print("cmd=");
    DEBUG_SERIAL.println((uint8_t)cmd, HEX);
    DEBUG_SERIAL.print("len=");
    DEBUG_SERIAL.println(len);
    for (uint8_t i = 0; i < len; i++)
    {
      DEBUG_SERIAL.print(buff[i], HEX);
      DEBUG_SERIAL.print(' ');
    }
    DEBUG_SERIAL.println();
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
    default:
      break;
    }
  });

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  /* Init i2c bus (Arduino) */
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  /* Init i2c bus (Fast Wire) */
  Fastwire::setup(400, true);
#endif /* I2CDEV_IMPLEMENTATION */

#ifdef GPS_SERIAL
  GPS_SERIAL.begin(GPS_BAUD);
#endif /* GPS_SERIAL */

#ifndef DISABLE_IMU
  /* Init IMU */
  g_imu.initialize();
#ifdef DEBUG
  DEBUG_SERIAL.print("IMU init: ");
  DEBUG_SERIAL.println(g_imu.testConnection());
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
  DEBUG_SERIAL.print("Barometer init: ");
  DEBUG_SERIAL.println(g_barometer.testConnection());
#endif /* DEBUG */
#endif /* DISABLE_BARO */

  /* Init scheduler */
  g_scheduler.addTask(&taskBlink, Scheduler::HzToUsInterval(5.0f));
  g_scheduler.addTask(&taskMSP, 0);
#ifdef TEAPOT
  g_scheduler.addTask(&taskTeapot, Scheduler::HzToUsInterval(50.0f));
#endif
#ifndef DISABLE_IMU
  if (dmpStatus == 0)
    g_scheduler.addTask(&taskDMP, 0);
#endif /* DISABLE_IMU */
#ifndef DISABLE_BARO
  g_scheduler.addTask(&taskBarometer, Scheduler::HzToUsInterval(10.0f));
#endif /* DISABLE_BARO */
#ifdef DEBUG
  g_scheduler.addTask(&taskPrintData, Scheduler::HzToUsInterval(10.0f));
  g_scheduler.print(DEBUG_SERIAL);
#endif /* DEBUG */

  digitalWrite(LED_PIN, LOW);
#ifdef DEBUG
  DEBUG_SERIAL.println("Ready");
#endif /* DEBUG */
}

void loop()
{
  g_scheduler.loop();
}
