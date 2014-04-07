#include "servo.h"

//Servomotor control pulse period in milliseconds
#define SERVO_PULSE_PERIOD_MS 50
//max control pulse width
#define SERVO_MAX_PULSE_WIDTH_US 2000
//min control pulse width
#define SERVO_MIN_PULSE_WIDTH_US 1000


/*
ServoInitConnectivity(...)
ServoInitOptions
ServoInitPeryph



*/

gyro_error ServoInitConnectivity(servo_connectivity_conf* conn,
				 uint16_t pin,
				 GPIO_TypeDef* port,
				 TIM_TypeDef* tim)
{
	gyro_error err = NO_ERROR;
	if (conn == NULL || port == NULL || tim == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
	if (pin > 15) {
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}

	conn->servo_GPIO_PORT = port;
	conn->servo_PIN = pin;
	conn->servo_TIM = tim;

	switch(conn->servo_TIM) {
	case (TIM2):
		conn->timer_clock = RCC_APB1Periph_TIM2;
		conn->gpio_af = GPIO_AF_TIM2;
	break;
	case (TIM3):
		conn->timer_clock = RCC_APB1Periph_TIM3;
		conn->gpio_af = GPIO_AF_TIM3;
	break;
	case (TIM4):
		conn->timer_clock = RCC_APB1Periph_TIM4;
		conn->gpio_af = GPIO_AF_TIM4;
	break;
	case (TIM5):
		conn->timer_clock = RCC_APB1Periph_TIM5;
		conn->gpio_af = GPIO_AF_TIM5;
	break;
	case (TIM6):
		conn->timer_clock = RCC_APB1Periph_TIM6;
		conn->gpio_af = GPIO_AF_TIM6;
	break;
	case (TIM7):
		conn->timer_clock = RCC_APB1Periph_TIM7;
		conn->gpio_af = GPIO_AF_TIM7;
	break;
	default :
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}
	
	switch (conn->port_clock) {
	case (GPIOA) :
		conn->port_clock = RCC_AHB1Periph_GPIOA;
	break;
	case (GPIOB) :
		conn->port_clock = RCC_AHB1Periph_GPIOB;
	break;
	case (GPIOC) :
		conn->port_clock = RCC_AHB1Periph_GPIOC;
	break;
	case (GPIOD) :
		conn->port_clock = RCC_AHB1Periph_GPIOD;
	break;
	case (GPIOE) :
		conn->port_clock = RCC_AHB1Periph_GPIOE;
	break;
	case (GPIOF) :
		conn->port_clock = RCC_AHB1Periph_GPIOF;
	break;
	case (GPIOG) :
		conn->port_clock = RCC_AHB1Periph_GPIOG;
	break;
	case (GPIOH) :
		conn->port_clock = RCC_AHB1Periph_GPIOH;
	break;
	case (GPIOI) :
		conn->port_clock = RCC_AHB1Periph_GPIOI;
	break;
	default :
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}

	switch(conn->servo_PIN) {
	case (GPIO_Pin_0) :
		conn->pin_source = GPIO_PinSource0;
	break;
	case (GPIO_Pin_1) :
		conn->pin_source = GPIO_PinSource1;
	break;
	case (GPIO_Pin_2) :
		conn->pin_source = GPIO_PinSource2;
	break;
	case (GPIO_Pin_3) :
		conn->pin_source = GPIO_PinSource3;
	break;
	case (GPIO_Pin_4) :
		conn->pin_source = GPIO_PinSource4;
	break;
	case (GPIO_Pin_5) :
		conn->pin_source = GPIO_PinSource5;
	break;
	case (GPIO_Pin_6) :
		conn->pin_source = GPIO_PinSource6;
	break;
	case (GPIO_Pin_7) :
		conn->pin_source = GPIO_PinSource7;
	break;
	case (GPIO_Pin_8) :
		conn->pin_source = GPIO_PinSource8;
	break;
	case (GPIO_Pin_9) :
		conn->pin_source = GPIO_PinSource9;
	break;
	case (GPIO_Pin_10) :
		conn->pin_source = GPIO_PinSource10;
	break;
	case (GPIO_Pin_11) :
		conn->pin_source = GPIO_PinSource11;
	break;
	case (GPIO_Pin_12) :
		conn->pin_source = GPIO_PinSource12;
	break;
	case (GPIO_Pin_13) :
		conn->pin_source = GPIO_PinSource13;
	break;
	case (GPIO_Pin_14) :
		conn->pin_source = GPIO_PinSource14;
	break;
	case (GPIO_Pin_15) :
		conn->pin_source = GPIO_PinSource15;
	break;
	}

err_occured:
	return err;
}

gyro_error ServoInitOptions(servo_conf* conf,
			    uint32_t resolution)
{
	gyro_error err = NO_ERROR;
	if (conf == NULL || conf->connectivity == NULLs) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
	if (resolution == 0) {
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}
	conf->max_resolution = resolution;
//SystemCoreClock / prescaler = delta_pulse *1000000 / conf->max_resolution
	conf->connectivity.prescaler = SystemCoreClock / 
	((SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US)*1000000) *
	conf->max_resolution;
err_occured:
	return err;
}

gyro_error ServoInitPeriph(servo_conf* conf)
{
	gyro_error err = NO_ERROR;
	/*TODO: look at maximum prescaler size(16 or 32 bits)*/
	uint32_t prescaler_val;
	if (conf == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
	GPIO_InitTypeDef GPIO_InitStructure;
	/*TIM clock enable*/
	RCC_APB1PeriphClockCmd(conf->connectivity.timer_clock, ENABLE);
	/* Port clock enable */
	RCC_AHB1PeriphClockCmd(conf->connectivity.port_clock, ENABLE);

	GPIO_InitStructure.GPIO_Pin = conf->connectivity.servo_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(conf->connectivity.servo_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(conf->connectivity.servo_GPIO_PORT,
			 conf->connectivity.pin_source,
			 conf->connectivity.gpio_af);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	


err_occured:
	return err;
}