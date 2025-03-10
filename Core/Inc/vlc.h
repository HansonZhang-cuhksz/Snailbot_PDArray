#ifndef VLC_H
#define VLC_H

#include "main.h"
#include "dsp_task.h"

extern uint8_t recv_byte;
extern uint8_t recv_byte_lock;
extern uint16_t recv_pd_idx;

extern void vlc_init(void);
extern void vlc_read(uint16_t adc_value);
extern uint8_t vlc_read_byte(void);
extern void vlc_task(void);

#endif
