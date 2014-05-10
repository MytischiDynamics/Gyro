#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx.h"
#include "gyro_error.h"

#define NULL (void*)0

typedef struct {
	uint16_t servo_PIN;
	GPIO_TypeDef* servo_GPIO_PORT;
	TIM_TypeDef* servo_TIM;
	uint32_t channel;
	uint32_t timer_clock;
	uint32_t port_clock;
	uint8_t pin_source;
	uint8_t gpio_af;
	uint16_t prescaler;

	struct_init_status init_status;
} servo_connectivity_conf;

typedef struct {
	servo_connectivity_conf connectivity;
	uint32_t resolution;
	uint32_t speed;

	struct_init_status init_status;
} servo_conf;

gyro_error ServoInit(servo_conf* conf,
		     uint16_t pin,
		     GPIO_TypeDef* port,
		     TIM_TypeDef* tim,
		     uint32_t channel);

gyro_error ServoSetSpeed(servo_conf* conf, int32_t speed);

#endif