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
  int i;
  int16_t ax, ay, az, gx, gy, gz;
  float ax2, ay2, az2, gx2, gy2, gz2;

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

    mpu6000_get_motion_6(&ax, &ay, &az, &gx, &gy, &gz);
    printf("acc(x/y/z): %05d, %05d, %05d\n", ax, ay, az);
    printf("gyr(x/y/z): %05d, %05d, %05d\n", gx, gy, gz);

    gx2 = gx / 131.0f;
    gy2 = gy / 131.0f;
    gz2 = gz / 131.0f;
    ax2 = ax / 2048.0f;
    ay2 = ay / 2048.0f;
    az2 = az / 2048.0f;
    printf("acc(x/y/z): %f, %f, %f\n", ax2, ay2, az2);
    printf("gyr(x/y/z): %f, %f, %f\n", gx2, gy2, gz2);

    MadgwickAHRSupdateIMU(gx2, gy2, gz2, ax2, ay2, az2);
    printf("q(w/x/y/z): %f, %f, %f, %f\n", q0, q1, q2, q3);

    msleep(100);
  }
}
