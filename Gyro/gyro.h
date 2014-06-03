#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "gyro_error.h"
#include "L3G4200D/L3G4200D.h"
#include "Servo/servo.h"
#include "DataBuffer/data_buffer.h"

typedef struct {
	l3g4200d_conf gyroscope;
	servo_conf servo;
	data_buffer vel_data;
//	int16_t vels[5000];
//	int vels_count;
} gyro_data_t;

gyro_data_t* get_gyro_data();

#endif
