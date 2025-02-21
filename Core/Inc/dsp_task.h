#ifndef DSP_TASK_H
#define DSP_TASK_H

#include "main.h"
#include "string.h"

typedef struct
{
	uint32_t sum;
	uint16_t data[50];
	uint8_t index;
} dsp_avg_data_t;

extern uint8_t dsp_task_watchdog;
extern uint8_t dsp_tick;

extern uint8_t selected_idx;
extern dsp_avg_data_t dsp_avg_data[128];
extern uint16_t dsp_buf[400];
extern uint16_t dsp_buf_idx;

extern uint16_t get_dsp_avg(dsp_avg_data_t* data);
extern void update_dsp_avg(dsp_avg_data_t* data, uint16_t value);

extern void DSP_init(void);
extern void DSP_task(void);

#endif
