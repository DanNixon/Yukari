#include "mpu6000.h"

#include <stdint.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/spi.h>

void mpu6000_init(void)
{
  rcc_periph_clock_enable(RCC_GPIOA | RCC_GPIOC | RCC_SPI1);

  /* Chip select pin */
  gpio_mode_setup(MPU6000_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MPU6000_CS_PIN);
  gpio_set(MPU6000_CS_PORT, MPU6000_CS_PIN);

  /* TODO */
}
