#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "MadgwickAHRS.h"
#include "clock.h"
#include "console.h"
#include "mpu6000.h"

#define LED0_PORT GPIOB
#define LED0_PIN GPIO5

typedef struct
{
  uint8_t header;
  uint8_t length;
  uint16_t q_w;
  uint16_t q_x;
  uint16_t q_y;
  uint16_t q_z;
  uint16_t d_x;
  uint16_t d_y;
  uint16_t d_z;
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
  u.values.q_w = q0 * 1000;
  u.values.q_x = q1 * 1000;
  u.values.q_y = q2 * 1000;
  u.values.q_z = q3 * 1000;
  u.values.d_x = 0;
  u.values.d_y = 0;
  u.values.d_z = 0;
  u.values.checksum = 0;
  u.values.padding = 0;

  for (i = 0; i < 15; i++)
    u.values.checksum ^= u.data[i];

  console_write(u.data, sizeof(Packet));
}

static void setup_leds(void)
{
  gpio_mode_setup(LED0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED0_PIN);
  gpio_set(LED0_PORT, LED0_PIN);
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

  setup_leds();
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

    switch (console_rx_command)
    {
    case 'p':
      send_data_packet();
      printf("\n");
      console_rx_command = '\0';
      break;
    case 'd':
      printf("samples=%lld\n", mpu6000_samples);
      printf("gyr(x/y/z): %f, %f, %f\n", mpu6000_axis[0], mpu6000_axis[1], mpu6000_axis[2]);
      printf("acc(x/y/z): %f, %f, %f\n", mpu6000_axis[3], mpu6000_axis[4], mpu6000_axis[5]);
      printf("q(w/x/y/z): %f, %f, %f, %f\n", q0, q1, q2, q3);
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
