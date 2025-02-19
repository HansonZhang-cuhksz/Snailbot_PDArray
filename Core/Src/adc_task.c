#include "adc_task.h"

uint16_t adc_buf1[2];
uint16_t adc_buf2[3];
uint16_t adc_buf3[1];
uint16_t adc_buf4[1];
uint16_t adc_buf5[1];
uint16_t adc_values[8];
uint8_t adc_done, done_count = 0;
uint8_t watcher = 0, watcher_bool = 0;
uint16_t VLP_value[128];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1)
    {
      adc_values[6] = adc_buf1[0];
      adc_values[1] = adc_buf1[1];
      adc_done |= 1;
    }
    else if(hadc->Instance == ADC2)
    {
      adc_values[0] = adc_buf2[0];
      adc_values[3] = adc_buf2[1];
      adc_values[2] = adc_buf2[2];
      adc_done |= 2;
    }
    else if(hadc->Instance == ADC3)
    {
      adc_values[7] = adc_buf3[0];
      adc_done |= 4;
    }
    else if(hadc->Instance == ADC4)
    {
      adc_values[4] = adc_buf4[0];
      adc_done |= 8;
    }
    else if(hadc->Instance == ADC5)
    {
      adc_values[5] = adc_buf5[0];
      adc_done |= 16;
    }
    done_count++;
}

void get_VLP_value(uint16_t *VLP_value)
{
  for(uint8_t i = 0; i < 16; i++)
  {
    for(uint8_t switch_num = 0; switch_num < 8; switch_num++)
    {
      set_switches(switch_num, i);
    }

    adc_done = 0x00;
    done_count = 0;

    for(uint8_t switch_num = 0; switch_num < 8; switch_num++)
    {
      while(get_switches(switch_num) != i);
    }
    
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf1, 2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buf2, 3);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_buf3, 1);
    HAL_ADC_Start_DMA(&hadc4, (uint32_t *)adc_buf4, 1);
    HAL_ADC_Start_DMA(&hadc5, (uint32_t *)adc_buf5, 1);

    while(adc_done != 0x0F)
		{
			if(adc_done == 0x1E)
        watcher = 1;
      else if (adc_done == 0x1D)
        watcher = 2;
      else if (adc_done == 0x1B)
        watcher = 3;
      else if (adc_done == 0x17)
        watcher = 4;
      else if (adc_done == 0x0F)
        watcher = 5;
      else
        watcher = 6;
		}
    
		watcher_bool = 1;
    for(uint8_t j = 0; j < 8; j++)
    {
      VLP_value[i + j * 16] = adc_values[j];//filter_update(&filter[i + j * 16], adc_values[j]);
    }
  }
}

void ADC_task(void)
{
	adc_task_watchdog = 0;
	get_VLP_value(VLP_value);
  switch_formulate(VLP_value, (uint16_t *)VLP_packet.luminance);
}
