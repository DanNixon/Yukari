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

#define LED0_PORT GPIOB
#define LED0_PIN GPIO5

static void setup_leds(void)
{
  gpio_mode_setup(LED0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED0_PIN);
  gpio_set(LED0_PORT, LED0_PIN);
}

int main(void)
{
  int i;
  int16_t ax, ay, az, gx, gy, gz;

  clock_setup();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_USART3);

  setup_leds();
  console_setup(115200);
  mpu6000_init();

  printf("Hello world.\n");
  for (i = 0; i < 5; i++)
  {
    gpio_toggle(LED0_PORT, LED0_PIN);
    msleep(5);
  }

  i = 0;
  while (1)
  {
    gpio_toggle(LED0_PORT, LED0_PIN);
    printf("i=%d\n", i++);
    printf("millis()=%lld\n", millis());
    printf("micros()=%lld\n", micros());

    mpu6000_get_motion_6(&ax, &ay, &az, &gx, &gy, &gz);
    printf("acc(x/y/z): %05d, %05d, %05d\n", ax, ay, az);
    printf("gyr(x/y/z): %05d, %05d, %05d\n", gx, gy, gz);

    msleep(8);
  }
}
