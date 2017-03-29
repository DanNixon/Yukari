#include <limits.h>

#include <IMU_MPU9150.h>

#include <MSP.h>
#include <Wire.h>
#include <helper_3dmath.h>

#define LED_PIN 13

MSP g_msp(Serial1);

IMU_MPU9150 g_imu;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(9600);

  Serial1.begin(115200);
  g_msp.begin();

  // Wait for a serial connection
  while (!Serial)
    delay(5);

  // Init i2c bus
  Wire.begin();

  // Init IMU
  bool imuInitResult = g_imu.init();
  Serial.print("IMU init: ");
  Serial.println(imuInitResult);

  Serial.println("Up");
  digitalWrite(LED_PIN, LOW);
}

uint32_t lastSampleTime = 0;

void loop()
{
  g_msp.loop();

  static uint32_t now;

  now = micros();
  if (now - lastSampleTime > 1000 / 250) // 4kHz
    g_imu.sample();

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

  delay(10);
}
