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

void taskIMU()
{
  g_imu.sample();
}

void taskMSP()
{
  g_msp.loop();
}

void taskDebugIMU()
{
  IMU::IMUData d = g_imu.filteredData();

  Serial.print("a/g/m:\t");
  Serial.print(d.acc[0]);
  Serial.print("\t");
  Serial.print(d.acc[1]);
  Serial.print("\t");
  Serial.print(d.acc[2]);
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

  Serial.printf("Delta: %d\n", g_scheduler.getDelta(imuTask));
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);

  Serial.begin(9600);

  // Init scheduler
  imuTask = g_scheduler.addTask(&taskIMU, 1000); // 1kHz
  // g_scheduler.addTask(&taskMSP, 500);
  g_scheduler.addTask(&taskDebugIMU, 50000);

  // Init MSP
  g_msp.setOnMessage([](MSP::Direction dir, uint8_t cmd, uint8_t *buff, uint8_t len) {
    Serial.println((uint8_t)dir);
    Serial.println(cmd);
    Serial.println(buff[0]);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  });

  // Wait for a serial connection
  while (!Serial)
    delay(5);

  // Init i2c bus
  Wire.begin();

  // Init IMU
  bool imuInitResult = g_imu.init();
  // Serial.print("IMU init: ");
  // Serial.println(imuInitResult);

  // Serial.println("Up");
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  g_scheduler.loop();
}
