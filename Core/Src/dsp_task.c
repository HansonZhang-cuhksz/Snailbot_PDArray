#include "dsp_task.h"

#define DSP_SAMPLE_RATE 1200

uint8_t dsp_tick;

uint8_t selected_idx = 0xFF;
dsp_avg_data_t dsp_avg_data[128];

uint16_t dsp_buf[DSP_SAMPLE_COUNT];
uint16_t dsp_buf_idx;

uint8_t cumulating;
uint8_t achieved399 = 0;

arm_rfft_fast_instance_f32 S;
float32_t input[DSP_SAMPLE_COUNT];
float32_t output[DSP_SAMPLE_COUNT];

float32_t magnitudes[DSP_SAMPLE_COUNT / 2];

float32_t top3_freqs[3];

uint8_t arm_math_valid = 0;

uint16_t get_dsp_avg(dsp_avg_data_t* data)
{
	return data->sum / 50;
}

void init_dsp_avg(dsp_avg_data_t* data)
{
	data->sum = 0;
	data->index = 0;
}

void update_dsp_avg(dsp_avg_data_t* data, uint16_t value)
{
	data->sum -= data->data[data->index];
	data->data[data->index] = value;
	data->sum += value;
	data->index = (data->index + 1) % 50;
}

void export_dsp_data(dsp_avg_data_t* data, uint16_t* buffer)
{
	memcpy(buffer, &data->data[data->index], (50 - data->index) * sizeof(uint16_t));
	memcpy(&buffer[50 - data->index], data->data, data->index * sizeof(uint16_t));
}

void DSP_process(uint16_t* input_raw)
{
    // Convert input data to float32
    for (uint16_t i = 0; i < DSP_SAMPLE_COUNT; i++)
    {
        input[i] = (float32_t)input_raw[i];
    }

    // Perform the FFT
    if (arm_rfft_fast_init_f32(&S, DSP_SAMPLE_COUNT) != ARM_MATH_SUCCESS)
    {
		arm_math_valid = 0;
        return;
    }
	arm_math_valid = 1;
    arm_rfft_fast_f32(&S, input, output, 0);

    // Calculate magnitudes of the FFT output
    for (uint16_t i = 0; i < DSP_SAMPLE_COUNT / 2; i++)
    {
        magnitudes[i] = sqrtf(output[2 * i] * output[2 * i] + output[2 * i + 1] * output[2 * i + 1]);
	}

	memcpy((void*)comm_packet.dsp_data, (void*)magnitudes, DSP_SAMPLE_COUNT * sizeof(uint32_t) / 2);
}

void DSP_init(void)
{
	dsp_buf_idx = 0;
	cumulating = 0;
	for (int i = 0; i < 128; i++)
	{
		init_dsp_avg(&dsp_avg_data[i]);
	}
}

void DSP_task(void)
{
	dsp_task_watchdog = 0;

	uint16_t min = 0xFFFF;
	uint8_t min_idx = 0xFF;
	for (int i = 32; i < 48; i++)
	{
		uint16_t avg = get_dsp_avg(&dsp_avg_data[i]);
		if (avg < min && avg > 0x0200)
		{
			min = avg;
			min_idx = i;
		}
	}
	for (int i = 80; i < 95; i++)
	{
		uint16_t avg = get_dsp_avg(&dsp_avg_data[i]);
		if (avg < min && avg > 0x0050)
		{
			min = avg;
			min_idx = i;
		}
	}

	// Start dsp capture
	if (min < 0x0E00 && min_idx != 0xFFFF && selected_idx == 0xFF)
	{
		dsp_buf_idx = 50;
		selected_idx = 37;
		export_dsp_data(&dsp_avg_data[selected_idx], dsp_buf);
	}

	// Update dsp to serial
	if (dsp_buf_idx >= DSP_SAMPLE_COUNT)
	{
		achieved399 = 1;
		selected_idx = 0xFF;
		dsp_buf_idx = 0;
		DSP_process(dsp_buf);
	}
}
