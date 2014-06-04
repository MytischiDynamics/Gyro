#ifndef DATABUFFER_H
#define DATABUFFER_H

#include "gyro_error.h"
#include "stm32f4xx.h"

#define NULL (void*)0

typedef struct {
	int16_t* buffer;
	int16_t* current_session_buffer_start;
	int16_t* previous_session_buffer_start;
	int counter;
	int block_ready;
	int size;
} data_buffer;

gyro_error InitDataBuffer(data_buffer* db, uint16_t block_size, int16_t* buffer);
gyro_error WriteValue(data_buffer *db, int16_t value);







#endif