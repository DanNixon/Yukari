#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "MadgwickAHRS.h"
#include "clock.h"
#include "console.h"
#include "leds.h"
#include "mpu6000.h"

typedef struct
{
  uint8_t header;
  uint8_t length;
  int16_t q_w;
  int16_t q_x;
  int16_t q_y;
  int16_t q_z;
  int16_t d_x;
  int16_t d_y;
  int16_t d_z;
  uint8_t checksum;
  uint8_t padding;
} Packet;

static void send_data_packet(void)
{
  int i;

  union {
    Packet values;
    uint8_t data[sizeof(Packet)];
  } u;

  u.values.header = '#';
  u.values.length = sizeof(Packet);
  u.values.q_w = q0 * 10000.0f;
  u.values.q_x = q1 * 10000.0f;
  u.values.q_y = q2 * 10000.0f;
  u.values.q_z = q3 * 10000.0f;
  u.values.d_x = mpu6000_world_displacement[0] * 1000.0f;
  u.values.d_y = mpu6000_world_displacement[1] * 1000.0f;
  u.values.d_z = mpu6000_world_displacement[2] * 1000.0f;
  u.values.checksum = 0;
  u.values.padding = 12;

  for (i = 0; i < 15; i++)
    u.values.checksum ^= u.data[i];

  console_write(u.data, sizeof(Packet));
}

/*
 * Print out data in CSV format.
 * Good for plotting with the likes of serialplot.
 * https://bitbucket.org/hyOzd/serialplot
 *
 *  1) linear acceleration - x
 *  2) linear acceleration - y
 *  3) linear acceleration - z
 *  4) gravity vector - x
 *  5) gravity vector - y
 *  6) gravity vector - z
 *  7) world acceleration - x
 *  8) world acceleration - y
 *  9) world acceleration - z
 * 10) world velocity - x
 * 11) world velocity - y
 * 12) world velocity - z
 * 13) world displacement - x
 * 14) world displacement - y
 * 15) world displacement - z
 */
static void send_logging_packet(void)
{
  /* Forgive me for this shitty printf */
  printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", mpu6000_linear_accel[0], mpu6000_linear_accel[1],
         mpu6000_linear_accel[2], mpu6000_gravity[0], mpu6000_gravity[1], mpu6000_gravity[2],
         mpu6000_world_accel[0], mpu6000_world_accel[1], mpu6000_world_accel[2],
         mpu6000_world_velocity[0], mpu6000_world_velocity[1], mpu6000_world_velocity[2],
         mpu6000_world_displacement[0], mpu6000_world_displacement[1],
         mpu6000_world_displacement[2]);
}

int main(void)
{
  uint64_t last_sample_time, now;

  clock_setup();

  rcc_periph_clock_enable(RCC_SYSCFG);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_USART3);

  leds_init();
  console_setup(115200);
  mpu6000_init();

  msleep(500);

  printf("Calibrating IMU...\n");
  mpu6000_calibrate();
  printf("Calibration done\n");

  printf("Packet len: %d\n", sizeof(Packet));

  printf("Hello world.\n");
  gpio_set(LED0_PORT, LED0_PIN);
  msleep(500);
  gpio_clear(LED0_PORT, LED0_PIN);

  last_sample_time = micros();
  mpu6000_last_integration_time = micros();
  while (1)
  {
    now = micros();

    if (now - last_sample_time >= 125)
    {
      gpio_toggle(LED0_PORT, LED0_PIN);
      mpu6000_sample();
      last_sample_time = now;
    }

    if (mpu6000_samples_acc >= MPU6000_ACC_SAMPLES)
    {
      mpu6000_position_update();

      /* Uncomment this for logging with serialplot */
      /* send_logging_packet(); */
    }

    switch (console_rx_command)
    {
    case 'p':
      send_data_packet();
      console_rx_command = '\0';
      break;
    case 'r':
      mpu6000_reset_integrators();
      console_rx_command = '\0';
      break;
    case 'c':
      mpu6000_calibrate();
      console_rx_command = '\0';
      break;
    case 'v':
      printf("Accel. calib. values: %d, %d, %d\n", mpu6000_acc_calib[0], mpu6000_acc_calib[1],
             mpu6000_acc_calib[2]);
      printf("Gyro. calib. values: %d, %d, %d\n", mpu6000_gyr_calib[0], mpu6000_gyr_calib[1],
             mpu6000_gyr_calib[2]);
      console_rx_command = '\0';
      break;
    case 'd':
      printf("samples=%lld\n", mpu6000_samples);
      printf("gyr(x/y/z): %f, %f, %f\n", mpu6000_axis[0], mpu6000_axis[1], mpu6000_axis[2]);
      printf("acc(x/y/z): %f, %f, %f\n", mpu6000_axis[3], mpu6000_axis[4], mpu6000_axis[5]);
      printf("q(w/x/y/z): %f, %f, %f, %f\n", q0, q1, q2, q3);
      printf("grav(x/y/z): %f, %f, %f\n", mpu6000_gravity[0], mpu6000_gravity[1],
             mpu6000_gravity[2]);
      printf("lacc(x/y/z): %f, %f, %f\n", mpu6000_linear_accel[0], mpu6000_linear_accel[1],
             mpu6000_linear_accel[2]);
      printf("wacc(x/y/z): %f, %f, %f\n", mpu6000_world_accel[0], mpu6000_world_accel[1],
             mpu6000_world_accel[2]);
      printf("wvel(x/y/z): %f, %f, %f\n", mpu6000_world_velocity[0], mpu6000_world_velocity[1],
             mpu6000_world_velocity[2]);
      printf("wdisp(x/y/z): %f, %f, %f\n", mpu6000_world_displacement[0],
             mpu6000_world_displacement[1], mpu6000_world_displacement[2]);
      console_rx_command = '\0';
      break;
    case 'u':
      printf("millis()=%lld\n", millis());
      printf("micros()=%lld\n", micros());
      console_rx_command = '\0';
      break;
    case 's':
      send_logging_packet();
      console_rx_command = '\0';
      break;
    default:
      break;
    }
  }
}
