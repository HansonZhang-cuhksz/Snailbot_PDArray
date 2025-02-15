#ifndef SWITCH_H
#define SWITCH_H

#include "stm32g4xx_hal.h"
#include <string.h>

extern void set_switches(uint8_t switch_num, uint8_t value);
extern void switch_formulate(uint16_t* in, uint16_t* out);

#endif // SWITCH_H
