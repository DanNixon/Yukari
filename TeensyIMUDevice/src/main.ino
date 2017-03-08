#include <I2Cdev.h>
#include <MPU9150.h>
#include <MSP.h>
#include <MSP_Constants.h>
#include <MSP_Data.h>
#include <MSP_Processors.h>
#include <Wire.h>
#include <helper_3dmath.h>

#define LED_PIN 13

MSP g_msp(Serial1);

MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(9600);

  Serial1.begin(115200);
  g_msp.begin();

  while (!Serial)
    delay(5);

  Wire.begin();

  Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful"
                                               : "MPU9150 connection failed");

  Serial.println("Up");
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  /* g_msp.loop(); */

  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Serial.print("a/g/m:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t");
  Serial.print(mx);
  Serial.print("\t");
  Serial.print(my);
  Serial.print("\t");
  Serial.print(mz);

  Serial.print("\n");

  delay(10);
}
