#include <stdint.h>
#include <stdlib.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/spi.h>

#define LED0_PORT GPIOB
#define LED0_PIN GPIO5

#define MPU6000_CS_PORT GPIOA
#define MPU6000_CS_PIN GPIO4

#define MPU6000_INT_PORT GPIOC
#define MPU6000_INT_PIN GPIO4

#define MPU6000_SPI_INSTANCE SPI1

static void setup_main_clock(void)
{
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void setup_peripheral_clocks(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SPI1);
}

static void setup_leds(void)
{
  gpio_mode_setup(LED0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED0_PIN);
  gpio_set(LED0_PORT, LED0_PIN);
}

int main(void)
{
  int i;

  setup_main_clock();
  setup_peripheral_clocks();
  setup_leds();

  while (1)
  {
    gpio_toggle(GPIOB, LED0_PIN);

    for (i = 0; i < 6000000; i++)
    {
      __asm__("nop");
    }
  }
}
