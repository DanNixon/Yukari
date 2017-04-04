#include <limits.h>

#include <MPU9150_9Axis_MotionApps41.h>
#include <helper_3dmath.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <MSP.h>
#include <Scheduler.h>

#define LED_PIN 13
#define MPU_INTERRUPT_PIN 20

Scheduler g_scheduler;

MSP g_msp(Serial);

MPU9150 g_imu;
uint16_t g_dmpFIFOPacketSize;
uint16_t g_dmpFIFOSize;
uint8_t g_dmpFIFOBuffer[64];
bool g_dmpReady = false;
volatile bool g_mpuInterrupt = false;

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL

void dmpDataReady()
{
  g_mpuInterrupt = true;
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  /* Init serial */
  Serial.begin(9600);
  while (!Serial)
    delay(5);

  /* Init scheduler */
  /* g_scheduler.addTask(&taskMSP, Scheduler::HzToUsInterval(50.0f)); */
  g_scheduler.print(Serial);

  /* Init MSP */
  g_msp.setOnMessage([](MSP::Direction dir, uint8_t cmd, uint8_t *buff, uint8_t len)
                     {
                       Serial.println((uint8_t)dir);
                       Serial.println(cmd);
                       Serial.println(buff[0]);
                       digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                     });

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  /* Init i2c bus (Arduino) */
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  /* Init i2c bus (Fast Wire) */
  Fastwire::setup(400, true);
#endif

  /* Init IMU */
  uint8_t status;

  g_imu.initialize();
  Serial.print("IMU init: ");
  Serial.println(g_imu.testConnection());

  pinMode(MPU_INTERRUPT_PIN, INPUT);

  status = g_imu.dmpInitialize();
  if (status == 0)
  {
    g_imu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    status = g_imu.getIntStatus();

    g_dmpFIFOPacketSize = g_imu.dmpGetFIFOPacketSize();
    g_dmpReady = true;
  }

  Serial.println("Up");
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  if (!g_dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!g_mpuInterrupt && g_dmpFIFOSize < g_dmpFIFOPacketSize)
  {
    /* g_scheduler.loop(); */
  }

  // reset interrupt flag and get INT_STATUS byte
  g_mpuInterrupt = false;
  uint8_t mpuIntStatus = g_imu.getIntStatus();

  // get current FIFO count
  g_dmpFIFOSize = g_imu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || g_dmpFIFOSize == 1024)
  {
    // reset so we can continue cleanly
    g_imu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (g_dmpFIFOSize < g_dmpFIFOPacketSize)
      g_dmpFIFOSize = g_imu.getFIFOCount();

    // read a packet from FIFO
    g_imu.getFIFOBytes(g_dmpFIFOBuffer, g_dmpFIFOPacketSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    g_dmpFIFOSize -= g_dmpFIFOPacketSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    g_imu.dmpGetQuaternion(&q, g_dmpFIFOBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    g_imu.dmpGetQuaternion(&q, g_dmpFIFOBuffer);
    g_imu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    g_imu.dmpGetQuaternion(&q, g_dmpFIFOBuffer);
    g_imu.dmpGetGravity(&gravity, &q);
    g_imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    g_imu.dmpGetQuaternion(&q, g_dmpFIFOBuffer);
    g_imu.dmpGetAccel(&aa, g_dmpFIFOBuffer);
    g_imu.dmpGetGravity(&gravity, &q);
    g_imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    g_imu.dmpGetQuaternion(&q, g_dmpFIFOBuffer);
    g_imu.dmpGetAccel(&aa, g_dmpFIFOBuffer);
    g_imu.dmpGetGravity(&gravity, &q);
    g_imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    g_imu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif
  }
}
