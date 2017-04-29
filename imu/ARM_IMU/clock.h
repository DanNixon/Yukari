#ifndef __CLOCK_H
#define __CLOCK_H

#include <stdint.h>

void msleep(uint64_t);
uint64_t millis(void);
uint64_t micros(void);
void clock_setup(void);

#endif /* __CLOCK_H */
