#ifndef ADC_TASK_H
#define ADC_TASK_H

#include "main.h"
#include "switch.h"
#include "dsp_task.h"

extern uint8_t adc_task_watchdog;

extern void ADC_task(void);
extern void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif // ADC_TASK_H
