#include <I2Cdev.h>

#include <MPU9150_9Axis_MotionApps41.h>
#include <helper_3dmath.h>

#include <MSP.h>
#include <Scheduler.h>

#define LED_PIN 13
#define MPU_INTERRUPT_PIN 20

Scheduler g_scheduler;

MSP g_msp(Serial);

MPU9150 g_imu;
uint16_t g_dmpFIFOPacketSize;
uint16_t g_dmpFIFOBufferSize;
uint8_t g_dmpFIFOBuffer[64];
bool g_dmpReady = false;
volatile bool g_mpuInterrupt = false;

Quaternion g_quat;
VectorFloat m_gravity;
VectorInt16 m_accel;
VectorInt16 m_realAccel;
VectorInt16 m_worldAccel;

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
  g_msp.setOnMessage([](MSP::Direction dir, MSP::Command cmd, uint8_t *buff, uint8_t len) {
    Serial.println((uint8_t)dir);
    Serial.println((uint8_t)cmd);
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

  g_imu.setFullScaleAccelRange(MPU9150_ACCEL_FS_16);
  g_imu.setFullScaleGyroRange(MPU9150_GYRO_FS_2000);

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
  while (!g_mpuInterrupt && g_dmpFIFOBufferSize < g_dmpFIFOPacketSize)
  {
    g_scheduler.loop();
  }

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
    Serial.println(F("FIFO overflow!"));
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

    // Device orientation as quaternion
    Serial.print("quat\t");
    Serial.print(g_quat.w);
    Serial.print("\t");
    Serial.print(g_quat.x);
    Serial.print("\t");
    Serial.print(g_quat.y);
    Serial.print("\t");
    Serial.println(g_quat.z);

    // Linear acceleration
    Serial.print("a\t");
    Serial.print(m_accel.x);
    Serial.print("\t");
    Serial.print(m_accel.y);
    Serial.print("\t");
    Serial.println(m_accel.z);

    // Linear acceleration without gravity
    Serial.print("areal\t");
    Serial.print(m_realAccel.x);
    Serial.print("\t");
    Serial.print(m_realAccel.y);
    Serial.print("\t");
    Serial.println(m_realAccel.z);

    // Linear acceleration without gravity and corrected for orientation
    Serial.print("aworld\t");
    Serial.print(m_worldAccel.x);
    Serial.print("\t");
    Serial.print(m_worldAccel.y);
    Serial.print("\t");
    Serial.println(m_worldAccel.z);
  }
}
