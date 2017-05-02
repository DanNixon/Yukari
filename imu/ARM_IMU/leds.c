#include "leds.h"


void leds_init(void)
{
  gpio_mode_setup(LED0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED0_PIN);
  gpio_set(LED0_PORT, LED0_PIN);
}
