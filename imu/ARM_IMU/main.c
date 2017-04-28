#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/spi.h>

#include "clock.h"
#include "console.h"
#include "mpu6000.h"

#define LED0_PORT_RCC RCC_GPIOB
#define LED0_PORT GPIOB
#define LED0_PIN GPIO5

static void setup_leds(void)
{
  rcc_periph_clock_enable(LED0_PORT_RCC);
  gpio_mode_setup(LED0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED0_PIN);
  gpio_set(LED0_PORT, LED0_PIN);
}

int main(void)
{
  clock_setup();

  setup_leds();
  console_setup(115200);
  mpu6000_init();

  printf("Hello world.\n");

  while (1)
  {
    gpio_toggle(LED0_PORT, LED0_PIN);
    msleep(500);
  }
}
