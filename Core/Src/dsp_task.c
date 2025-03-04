#include "dsp_task.h"

#define DSP_SAMPLE_RATE 1400
// #define K_DSP 300/262.5f

dsp_avg_data_t dsp_avg_data[128];
arm_rfft_fast_instance_f32 S;
uint16_t dsp_buf[128][DSP_SAMPLE_COUNT];
int16_t dsp_buf_idx[128] = {-1};	// -1 for not processing
float32_t input[DSP_SAMPLE_COUNT];
float32_t output[DSP_SAMPLE_COUNT];

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

void normalize_avg(float32_t* data)
{
	float32_t sum = 0;
	for (int i = 0; i < DSP_SAMPLE_COUNT; i++)
	{
		sum += data[i];
	}
	float32_t avg = sum / DSP_SAMPLE_COUNT;
	for (int i = 0; i < DSP_SAMPLE_COUNT; i++)
	{
		data[i] -= avg;
	}
}

uint16_t DSP_process(uint16_t* input_raw, uint16_t idx)
{
    // Convert input data to float32
    for (uint16_t i = 0; i < DSP_SAMPLE_COUNT; i++)
    {
        input[i] = (float32_t)input_raw[i];
    }

	normalize_avg(input);

    // Perform the FFT
    arm_rfft_fast_f32(&S, input, output, 0);

    // Calculate magnitudes of the FFT output
	uint16_t max_freq = 0;
	float32_t max_magnitude = 5000;
    for (uint16_t i = 5; i < DSP_SAMPLE_COUNT / 2; i++)
    {
        float32_t magnitude = sqrtf(output[2 * i] * output[2 * i] + output[2 * i + 1] * output[2 * i + 1]);
		if (magnitude > max_magnitude)
		{
			max_magnitude = magnitude;
			// max_freq = (float32_t)i * DSP_SAMPLE_RATE / DSP_SAMPLE_COUNT * K_DSP;
			max_freq = i;
		}
	}

	return max_freq;
}

void DSP_init(void)
{
	while(arm_rfft_fast_init_f32(&S, DSP_SAMPLE_COUNT) != ARM_MATH_SUCCESS);
	for (int i = 0; i < 128; i++)
	{
		init_dsp_avg(&dsp_avg_data[i]);
	}
}

void DSP_task(void)
{
	for (int i = 35; i < 39; i++)
	{
		uint16_t avg = get_dsp_avg(&dsp_avg_data[i]);
		if (avg < 0x0E00)
		{
			if (dsp_buf_idx[i] == -1)
			{
				dsp_buf_idx[i] = 50;
				export_dsp_data(&dsp_avg_data[i], dsp_buf[i]);
			}
		}
		else
		{
			VLP_packet.freq[i] = 0;
		}

		if (dsp_buf_idx[i] >= DSP_SAMPLE_COUNT)
		{
			VLP_packet.freq[i] = DSP_process(dsp_buf[i], i);
			dsp_buf_idx[i] = -1;
		}
	}
}
