#include <stdint.h>

uint32_t Cy_SysLib_EnterCriticalSection(void) {
  return 0;
}

void Cy_SysLib_ExitCriticalSection(uint32_t savedIntrStatus) {
  (void)savedIntrStatus;
}

__attribute__((weak)) void Cy_SysLib_DelayCycles(uint32_t cycles) {
  for (volatile uint32_t i = cycles; i; i--) continue;
}
