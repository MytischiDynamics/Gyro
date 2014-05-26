#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "gyro_error.h"
#include "L3G4200D/L3G4200D.h"
#include "Servo/servo.h"

typedef struct {
	l3g4200d_conf gyroscope;
	servo_conf servo;
} gyro_data_t;

gyro_data_t get_gyro_data();

#endif
