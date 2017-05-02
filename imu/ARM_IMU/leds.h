#ifndef __LEDS_H
#define __LEDS_H

#include <libopencm3/stm32/gpio.h>

#define LED0_PORT GPIOB
#define LED0_PIN GPIO5

void leds_init(void);

#endif /* __LEDS_H */
