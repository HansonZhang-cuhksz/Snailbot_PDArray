#include "switch.h"
#include "main.h"

#include <stdlib.h>

GPIO_PinState* set_pins(uint8_t value)
{
    GPIO_PinState* pins = (GPIO_PinState *)malloc(4 * sizeof(GPIO_PinState));
		pins[0] = (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    pins[1] = ((value >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    pins[2] = ((value >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    pins[3] = ((value >> 3) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return pins;
}

void set_switches(uint8_t switch_num, uint8_t value)
{
    GPIO_PinState* pins = set_pins(value);
    switch(switch_num)
    {
        case 0:
            HAL_GPIO_WritePin(SEL_0_0_15_GPIO_Port, SEL_0_0_15_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_0_15_GPIO_Port, SEL_1_0_15_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_0_15_GPIO_Port, SEL_2_0_15_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_0_15_GPIO_Port, SEL_3_0_15_Pin, pins[3]);
            break;
        case 1:
            HAL_GPIO_WritePin(SEL_0_16_31_GPIO_Port, SEL_0_16_31_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_16_31_GPIO_Port, SEL_1_16_31_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_16_31_GPIO_Port, SEL_2_16_31_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_16_31_GPIO_Port, SEL_3_16_31_Pin, pins[3]);
            break;
        case 2:
            HAL_GPIO_WritePin(SEL_0_32_47_GPIO_Port, SEL_0_32_47_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_32_47_GPIO_Port, SEL_1_32_47_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_32_47_GPIO_Port, SEL_2_32_47_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_32_47_GPIO_Port, SEL_3_32_47_Pin, pins[3]);
            break;
        case 3:
            HAL_GPIO_WritePin(SEL_0_48_63_GPIO_Port, SEL_0_48_63_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_48_63_GPIO_Port, SEL_1_48_63_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_48_63_GPIO_Port, SEL_2_48_63_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_48_63_GPIO_Port, SEL_3_48_63_Pin, pins[3]);
            break;
        case 4:
            HAL_GPIO_WritePin(SEL_0_64_79_GPIO_Port, SEL_0_64_79_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_64_79_GPIO_Port, SEL_1_64_79_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_64_79_GPIO_Port, SEL_2_64_79_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_64_79_GPIO_Port, SEL_3_64_79_Pin, pins[3]);
            break;
        case 5:
            HAL_GPIO_WritePin(SEL_0_80_95_GPIO_Port, SEL_0_80_95_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_80_95_GPIO_Port, SEL_1_80_95_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_80_95_GPIO_Port, SEL_2_80_95_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_80_95_GPIO_Port, SEL_3_80_95_Pin, pins[3]);
            break;
        case 6:
            HAL_GPIO_WritePin(SEL_0_96_111_GPIO_Port, SEL_0_96_111_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_96_111_GPIO_Port, SEL_1_96_111_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_96_111_GPIO_Port, SEL_2_96_111_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_96_111_GPIO_Port, SEL_3_96_111_Pin, pins[3]);
            break;
        case 7:
            HAL_GPIO_WritePin(SEL_0_112_127_GPIO_Port, SEL_0_112_127_Pin, pins[0]);
            HAL_GPIO_WritePin(SEL_1_112_127_GPIO_Port, SEL_1_112_127_Pin, pins[1]);
            HAL_GPIO_WritePin(SEL_2_112_127_GPIO_Port, SEL_2_112_127_Pin, pins[2]);
            HAL_GPIO_WritePin(SEL_3_112_127_GPIO_Port, SEL_3_112_127_Pin, pins[3]);
            break;
    }
		free(pins);
		//for(int i = 0; i < 100; i++){;}
}
