#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "gyro_error.h"
#include "L3G4200D/L3G4200D.h"
#include "Servo/servo.h"
#include "DataBuffer/data_buffer.h"
#include "arm_math.h"

typedef struct {
	l3g4200d_conf gyroscope;
	servo_conf servo;
	data_buffer vel_data;
	arm_fir_decimate_instance_q15 filter;
	arm_pid_instance_q15 regulator;
} gyro_data_t;

gyro_data_t* get_gyro_data();

#endif
