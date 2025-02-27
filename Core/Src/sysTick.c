#include "sysTick.h"


void SysTickConfig(void)
{
  /* Setup SysTick Timer for 1ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000)) //  1ms
  {
    /* Capture error */
    while (1)
      ;
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void set_systick_ms(uint32_t value)
{
    systick_ms = value; // Set the SysTick millisecond counter
}

uint32_t get_systick_ms(void)
{
    return systick_ms; // Return the current SysTick millisecond count
}
