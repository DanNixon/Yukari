/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>

#include "clock.h"

/* milliseconds since boot */
static volatile uint64_t system_micros;

/* Called when systick fires */
void sys_tick_handler(void)
{
  system_micros += 10;
}

/* simple sleep for delay milliseconds */
void msleep(uint64_t delay)
{
  uint32_t wake = system_micros + (delay * 1000);
  while (wake > system_micros)
    ;
}

uint64_t millis(void)
{
  return system_micros / 1000;
}

uint64_t micros(void)
{
  return system_micros;
}

/*
 * This function sets up both the base board clock rate
 * and a 10khz "system tick" count. The SYSTICK counter is
 * a standard feature of the Cortex-M series.
 */
void clock_setup(void)
{
  /* Base board frequency, set to 168Mhz */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  systick_set_reload(1680);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();

  /* this done last */
  systick_interrupt_enable();
}
