#include <limits.h>

#include <Wire.h>

#include <IMU_MPU9150.h>
#include <MSP.h>
#include <Scheduler.h>

#define LED_PIN 13

Scheduler g_scheduler;

MSP g_msp(Serial);
IMU_MPU9150 g_imu;

int8_t imuTask;

void taskSampleGyro()
{
  g_imu.sampleGyro();
}

void taskSampleAccel()
{
  g_imu.sampleAccel();
}

void taskSampleMag()
{
  g_imu.sampleMag();
}

void taskMSP()
{
  g_msp.loop();
}

void taskDebugIMU()
{
  IMU::IMUData d = g_imu.filteredData();

  Serial.print("a/g/m:\t");
  Serial.print(d.accel[0]);
  Serial.print("\t");
  Serial.print(d.accel[1]);
  Serial.print("\t");
  Serial.print(d.accel[2]);
  Serial.print("\t");
  Serial.print(d.gyro[0]);
  Serial.print("\t");
  Serial.print(d.gyro[1]);
  Serial.print("\t");
  Serial.print(d.gyro[2]);
  Serial.print("\t");
  Serial.print(d.mag[0]);
  Serial.print("\t");
  Serial.print(d.mag[1]);
  Serial.print("\t");
  Serial.print(d.mag[2]);

  Serial.print("\n");

  Serial.printf("Overrun: %d\n", g_scheduler.getOverrun(imuTask));
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);

  /* Init serial */
  Serial.begin(9600);
  while (!Serial)
    delay(5);

  /* Init scheduler */
  imuTask = g_scheduler.addTask(&taskSampleGyro, Scheduler::HzToUsInterval(GYRO_SAMPLE_FREQ));
  g_scheduler.addTask(&taskSampleAccel, Scheduler::HzToUsInterval(ACCEL_SAMPLE_FREQ));
  g_scheduler.addTask(&taskSampleMag, Scheduler::HzToUsInterval(MAG_SAMPLE_FREQ));
  /* g_scheduler.addTask(&taskMSP, Scheduler::HzToUsInterval(50.0f)); */
  g_scheduler.addTask(&taskDebugIMU, Scheduler::HzToUsInterval(25.0f));
  g_scheduler.print(Serial);

  /* Init MSP */
  g_msp.setOnMessage([](MSP::Direction dir, uint8_t cmd, uint8_t *buff, uint8_t len) {
    Serial.println((uint8_t)dir);
    Serial.println(cmd);
    Serial.println(buff[0]);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  });

  /* Init i2c bus */
  Wire.begin();

  /* Init IMU */
  bool imuInitResult = g_imu.init();
  Serial.print("IMU init: ");
  Serial.println(imuInitResult);

  Serial.println("Up");
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  g_scheduler.loop();
}
