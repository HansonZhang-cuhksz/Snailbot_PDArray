#include "vlc.h"

#define BAUD_RATE 50

#define QUEUE_SIZE 1000
#define queue_is_empty(queue) queue->start==queue->end
#define queue_is_full(queue) (queue->end+1)%QUEUE_SIZE==queue->start

typedef struct
{
	uint8_t bytes[QUEUE_SIZE];
	uint16_t start;
	uint16_t end;
} queue_t;

void init_queue(queue_t* queue)
{
	queue->start = 0;
	queue->end = 0;
}

uint8_t dequeue(queue_t* queue)
{
	if (queue_is_empty(queue))
	{
		return 0xFF;
	}
	
	uint8_t out = queue->bytes[queue->start];
	queue->start = (queue->start + 1) % QUEUE_SIZE;
	return out;
}

void enqueue(queue_t* queue, uint8_t byte)
{
	if (queue_is_full(queue))
	{
		dequeue(queue);
	}
	
	queue->bytes[queue->end] = byte;
	queue->end = (queue->end + 1) % QUEUE_SIZE;
}

queue_t recv_queue;

uint16_t recv_pd_idx = 0xFFFF;
#define BYTE_SIZE 9
uint16_t recv_buf[BYTE_SIZE];
uint8_t recv_buf_idx = 0;
uint8_t last_last_bit = 1;
uint8_t last_bit = 1;
uint8_t receiving = 0;

uint32_t receive_time = 0;
uint8_t curr_byte;

#define inverse(x) (x == 1) ? 0 : 1

uint8_t calculate_even_parity(uint8_t byte) {
  uint8_t parity = 0;
  for (uint8_t i = 0; i < 8; i++) {
    parity ^= (byte >> i) & 1;
  }
  return parity;
}

void vlc_init()
{
	init_queue(&recv_queue);
}

uint8_t vlc_read_byte()
{
	return dequeue(&recv_queue);
}

uint8_t vlc_tick = 0;
uint16_t cycle_sum = 0;
uint16_t thresh = 0x100;
void vlc_read(uint16_t adc_value)
{
	cycle_sum += adc_value;
	if (vlc_tick == (DSP_SAMPLE_RATE / BAUD_RATE))
	{
		// uint8_t bit = cycle_sum / (DSP_SAMPLE_RATE / BAUD_RATE) > thresh;
		uint8_t bit = adc_value > thresh;

		if (last_last_bit == 1 && last_bit == 0)
		{
			receiving = 1;
		}

		if (receiving)
		{
			recv_buf[recv_buf_idx] = bit;
			recv_buf_idx++;
			if (recv_buf_idx == BYTE_SIZE)
			{
				uint8_t byte = 0;
				for (uint8_t i = 0; i < 8; i++)
				{
					byte |= recv_buf[i] << i;
				}
				uint8_t even_parity = calculate_even_parity(byte);
				if (even_parity == recv_buf[8])
				{
					enqueue(&recv_queue, byte);
					receive_time++;
					curr_byte = byte;
				}
				receiving = 0;
				recv_buf_idx = 0;
			}
		}

		last_last_bit = last_bit;
		last_bit = bit;
		vlc_tick = 0;
	}
	vlc_tick++;
}

void vlc_task()
{
	// uint16_t new_recv_pd_idx = 0xFFFF;
	// uint16_t max_magnitude = 5000;
	// for (uint16_t idx = 0; idx < 128; idx++)
	// {
	// 	if (VLP_byte.freq[idx] > max_magnitude)
	// 	{
	// 		max_magnitude = VLP_byte.freq[idx];
	// 		new_recv_pd_idx = idx;
	// 	}
	// }
	// recv_pd_idx = new_recv_pd_idx;	
	recv_pd_idx = 37;
}
