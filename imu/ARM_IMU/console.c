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

#include "console.h"

#include <ctype.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "clock.h"

#define BUFLEN 127

static uint16_t start_ndx;
static uint16_t end_ndx;
static char buf[BUFLEN + 1];
#define buf_len ((end_ndx - start_ndx) % BUFLEN)

static inline int inc_ndx(int n)
{
  return ((n + 1) % BUFLEN);
}

static inline int dec_ndx(int n)
{
  return (((n + BUFLEN) - 1) % BUFLEN);
}

/* back up the cursor one space */
static inline void back_up(void)
{
  end_ndx = dec_ndx(end_ndx);
  usart_send_blocking(CONSOLE_UART, '\010');
  usart_send_blocking(CONSOLE_UART, ' ');
  usart_send_blocking(CONSOLE_UART, '\010');
}

/*
 * A buffered line editing function.
 */
void get_buffered_line(void)
{
  char c;

  if (start_ndx != end_ndx)
  {
    return;
  }
  while (1)
  {
    c = usart_recv_blocking(CONSOLE_UART);
    if (c == '\r')
    {
      buf[end_ndx] = '\n';
      end_ndx = inc_ndx(end_ndx);
      buf[end_ndx] = '\0';
      usart_send_blocking(CONSOLE_UART, '\r');
      usart_send_blocking(CONSOLE_UART, '\n');
      return;
    }
    /* ^H or DEL erase a character */
    if ((c == '\010') || (c == '\177'))
    {
      if (buf_len == 0)
      {
        usart_send_blocking(CONSOLE_UART, '\a');
      }
      else
      {
        back_up();
      }
      /* ^W erases a word */
    }
    else if (c == 0x17)
    {
      while ((buf_len > 0) && (!(isspace((int)buf[end_ndx]))))
      {
        back_up();
      }
      /* ^U erases the line */
    }
    else if (c == 0x15)
    {
      while (buf_len > 0)
      {
        back_up();
      }
      /* Non-editing character so insert it */
    }
    else
    {
      if (buf_len == (BUFLEN - 1))
      {
        usart_send_blocking(CONSOLE_UART, '\a');
      }
      else
      {
        buf[end_ndx] = c;
        end_ndx = inc_ndx(end_ndx);
        usart_send_blocking(CONSOLE_UART, c);
      }
    }
  }
}

/*
 * Called by libc stdio fwrite functions
 */
int _write(int fd, char *ptr, int len)
{
  int i = 0;

  if (fd > 2)
  {
    return -1;
  }
  while (*ptr && (i < len))
  {
    usart_send_blocking(CONSOLE_UART, *ptr);
    if (*ptr == '\n')
    {
      usart_send_blocking(CONSOLE_UART, '\r');
    }
    i++;
    ptr++;
  }
  return i;
}

/*
 * Called by the libc stdio fread fucntions
 *
 * Implements a buffered read with line editing.
 */
int _read(int fd, char *ptr, int len)
{
  int my_len;

  if (fd > 2)
  {
    return -1;
  }

  get_buffered_line();
  my_len = 0;
  while ((buf_len > 0) && (len > 0))
  {
    *ptr++ = buf[start_ndx];
    start_ndx = inc_ndx(start_ndx);
    my_len++;
    len--;
  }
  return my_len; /* return the length we got */
}

void console_setup(int baudrate)
{
  /* Setup GPIO pins for CONSOLE_UART transmit and receive. */
  gpio_mode_setup(CONSOLE_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CONSOLE_UART_RX_PIN | CONSOLE_UART_TX_PIN);
  gpio_set_af(CONSOLE_UART_PORT, GPIO_AF7, CONSOLE_UART_RX_PIN | CONSOLE_UART_TX_PIN);

  /* Setup CONSOLE_UART parameters. */
  usart_set_baudrate(CONSOLE_UART, baudrate);
  usart_set_databits(CONSOLE_UART, 8);
  usart_set_stopbits(CONSOLE_UART, USART_STOPBITS_1);
  usart_set_mode(CONSOLE_UART, USART_MODE_TX_RX);
  usart_set_parity(CONSOLE_UART, USART_PARITY_NONE);
  usart_set_flow_control(CONSOLE_UART, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(CONSOLE_UART);
}
