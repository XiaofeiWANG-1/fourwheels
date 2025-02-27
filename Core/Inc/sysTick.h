#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>
#include "main.h"

extern volatile uint32_t systick_ms; // Declare the SysTick counter variable

void SysTickConfig(void);
void set_systick_ms(uint32_t value); // Declare the function prototype
uint32_t get_systick_ms(void);

#endif // SYSTICK_H
