#include "uart_task.h"

void UART_task(void)
{
	VLP_packet.checksum = HAL_CRC_Calculate(&hcrc, (uint32_t*)&VLP_packet, sizeof(VLP_packet) - sizeof(uint32_t));
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&VLP_packet, sizeof(VLP_packet));
}
