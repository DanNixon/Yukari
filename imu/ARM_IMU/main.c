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
  /* u.values.d_x = mpu6000_world_displacement[0] * 1000.0f; */
  /* u.values.d_y = mpu6000_world_displacement[1] * 1000.0f; */
  /* u.values.d_z = mpu6000_world_displacement[2] * 1000.0f; */
  /* u.values.d_x = mpu6000_world_accel[0] * 1000.0f; */
  /* u.values.d_y = mpu6000_world_accel[1] * 1000.0f; */
  /* u.values.d_z = mpu6000_world_accel[2] * 1000.0f; */
  u.values.d_x = mpu6000_world_velocity[0] * 1000.0f;
  u.values.d_y = mpu6000_world_velocity[1] * 1000.0f;
  u.values.d_z = mpu6000_world_velocity[2] * 1000.0f;
  u.values.checksum = 0;
  u.values.padding = 12;

  for (i = 0; i < 15; i++)
    u.values.checksum ^= u.data[i];

  console_write(u.data, sizeof(Packet));
}

int main(void)
{
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

  printf("Packet len: %d\n", sizeof(Packet));

  printf("Hello world.\n");
  gpio_set(LED0_PORT, LED0_PIN);
  msleep(500);
  gpio_clear(LED0_PORT, LED0_PIN);

  while (1)
  {
    gpio_toggle(LED0_PORT, LED0_PIN);

    if (mpu6000_samples_acc >= MPU6000_ACC_SAMPLES)
      mpu6000_position_update();

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
    default:
      break;
    }
  }
}
