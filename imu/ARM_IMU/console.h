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

#ifndef __CONSOLE_H
#define __CONSOLE_H

/*
 * Some definitions of our console "functions" attached to the
 * USART.
 *
 * These define sort of the minimum "library" of functions which
 * we can use on a serial port. If you wish to use a different
 * USART there are several things to change:
 *	- CONSOLE_UART change this
 *	- Change the peripheral enable clock
 *	- add usartx_isr for interrupts
 *	- nvic_enable_interrupt(your choice of USART/UART)
 *	- GPIO pins for transmit/receive
 *		(may be on different alternate functions as well)
 */

#define CONSOLE_UART_PORT_RCC RCC_GPIOB
#define CONSOLE_UART_PORT GPIOB

#define CONSOLE_UART_RX_PIN GPIO11
#define CONSOLE_UART_TX_PIN GPIO10

#define CONSOLE_UART_RCC RCC_USART3
#define CONSOLE_UART USART3

int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);
void get_buffered_line(void);

void console_setup(int baudrate);

#endif /* __CONSOLE_H */