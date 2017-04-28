#include "mpu6000.h"

#include <stdint.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/stm32/f4/spi.h>

void mpu6000_init(void)
{
  /* Chip select pin */
  gpio_mode_setup(SPI1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MPU6000_CS_PIN);
  gpio_set(SPI1_PORT, MPU6000_CS_PIN);

  /* TODO */
}
