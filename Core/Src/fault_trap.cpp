#include "stm32g4xx.h"

// When any fault occurs, break into the debugger so you can see the backtrace.
__attribute__((noreturn)) void Fault_Loop(void) {
    __ASM volatile ("bkpt 0");  // debugger halts here
    while (1) { /* stay halted */ }
}

// These override the weak default handlers from startup code.
void HardFault_Handler(void)   { Fault_Loop(); }
void MemManage_Handler(void)   { Fault_Loop(); }
void BusFault_Handler(void)    { Fault_Loop(); }
void UsageFault_Handler(void)  { Fault_Loop(); }
