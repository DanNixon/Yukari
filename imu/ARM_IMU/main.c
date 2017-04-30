#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/spi.h>

#include "MadgwickAHRS.h"
#include "clock.h"
#include "console.h"
#include "mpu6000.h"

#define LED0_PORT GPIOB
#define LED0_PIN GPIO5

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

  printf("Hello world.\n");
  gpio_set(LED0_PORT, LED0_PIN);
  msleep(100);
  gpio_clear(LED0_PORT, LED0_PIN);

  while (1)
  {
    gpio_toggle(LED0_PORT, LED0_PIN);

    printf("samples=%lld\n", mpu6000_samples);
    printf("gyr(x/y/z): %f, %f, %f\n", mpu6000_axis[0], mpu6000_axis[1], mpu6000_axis[2]);
    printf("acc(x/y/z): %f, %f, %f\n", mpu6000_axis[3], mpu6000_axis[4], mpu6000_axis[5]);
    printf("q(w/x/y/z): %f, %f, %f, %f\n", q0, q1, q2, q3);

    msleep(100);
  }
}
