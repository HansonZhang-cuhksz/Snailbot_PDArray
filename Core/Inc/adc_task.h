#ifndef ADC_TASK_H
#define ADC_TASK_H

#include "main.h"
#include "switch.h"
#include "dsp_task.h"
#include "stm32g4xx_hal_rtc.h"

extern uint8_t adc_task_watchdog;

extern uint32_t time_diff;

extern void ADC_init(void);
extern void ADC_task(void);
extern void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif // ADC_TASK_H
