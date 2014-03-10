#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "gyro_error.h"
#include "L3G4200D/L3G4200D.h"

typedef struct
{
	l3g4200d_conf gyroscope;
} gyro_data_t;

#endif
