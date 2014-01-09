/*
 * imu_buffer.c
 *
 * Created: 1/21/2009 10:53:43 AM
 *  Author: Haichen Shen
 */

#include "twi_cdc_timer.h"

void init_buffer(imu_data_buffer *buffer)
{
	buffer->data = malloc(BUFFER_CAPACITY * sizeof(imu_data));
	buffer->capacity = BUFFER_CAPACITY;
	buffer->read_pt = 0;
	buffer->write_pt = 0;
}

void reset_buffer(imu_data_buffer *buffer)
{
	buffer->read_pt = 0;
	buffer->write_pt = 0;
}

int16_t buffer_size(imu_data_buffer *buffer)
{
	int16_t size = buffer->write_pt - buffer->read_pt;
	if (size < 0)
		size += buffer->capacity;

	return size;
}