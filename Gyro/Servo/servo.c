#include "servo.h"

//Servomotor control pulse period in milliseconds
#define SERVO_PULSE_PERIOD_MS 50
//max control pulse width
#define SERVO_MAX_PULSE_WIDTH_US 2000
//min control pulse width
#define SERVO_MIN_PULSE_WIDTH_US 1000

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

	conn->servo_GPIO_PORT = port;
	conn->servo_PIN = pin;
	conn->servo_TIM = tim;

	switch((uint32_t)conn->servo_TIM) {
	case ((uint32_t)TIM2):
		conn->timer_clock = RCC_APB1Periph_TIM2;
		conn->gpio_af = GPIO_AF_TIM2;
	break;
	case ((uint32_t)TIM3):
		conn->timer_clock = RCC_APB1Periph_TIM3;
		conn->gpio_af = GPIO_AF_TIM3;
	break;
	case ((uint32_t)TIM4):
		conn->timer_clock = RCC_APB1Periph_TIM4;
		conn->gpio_af = GPIO_AF_TIM4;
	break;
	case ((uint32_t)TIM5):
		conn->timer_clock = RCC_APB1Periph_TIM5;
		conn->gpio_af = GPIO_AF_TIM5;
	break;
	case ((uint32_t)TIM6):
		conn->timer_clock = RCC_APB1Periph_TIM6;
/*TODO : Find out where af of this timer*/
//		conn->gpio_af = GPIO_AF_TIM6;
	break;
	case ((uint32_t)TIM7):
		conn->timer_clock = RCC_APB1Periph_TIM7;
/*TODO : Find out where af of this timer*/
//		conn->gpio_af = GPIO_AF_TIM7;
	break;
	default :
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}
	
	switch ((uint32_t)conn->servo_GPIO_PORT) {
	case ((uint32_t)GPIOA) :
		conn->port_clock = RCC_AHB1Periph_GPIOA;
	break;
	case ((uint32_t)GPIOB) :
		conn->port_clock = RCC_AHB1Periph_GPIOB;
	break;
	case ((uint32_t)GPIOC) :
		conn->port_clock = RCC_AHB1Periph_GPIOC;
	break;
	case ((uint32_t)GPIOD) :
		conn->port_clock = RCC_AHB1Periph_GPIOD;
	break;
	case ((uint32_t)GPIOE) :
		conn->port_clock = RCC_AHB1Periph_GPIOE;
	break;
	case ((uint32_t)GPIOF) :
		conn->port_clock = RCC_AHB1Periph_GPIOF;
	break;
	case ((uint32_t)GPIOG) :
		conn->port_clock = RCC_AHB1Periph_GPIOG;
	break;
	case ((uint32_t)GPIOH) :
		conn->port_clock = RCC_AHB1Periph_GPIOH;
	break;
	case ((uint32_t)GPIOI) :
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

gyro_error ServoInitOptions(servo_conf* conf)
{
	gyro_error err = NO_ERROR;
	if (conf == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
//Make timer clock 1uS period
	conf->connectivity.prescaler = SystemCoreClock / 1000000;	
err_occured:
	return err;
}

gyro_error ServoSetDefaultTiming(servo_conf* conf)
{
	gyro_error err = NO_ERROR;
	if (conf == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
	if (conf->connectivity.prescaler == 0) {
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler = conf->connectivity.prescaler;
	TIM_TimeBaseStructure.TIM_Period = SERVO_PULSE_PERIOD_MS * 1000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(conf->connectivity.servo_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = (SERVO_MIN_PULSE_WIDTH_US + SERVO_MIN_PULSE_WIDTH_US) / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(conf->connectivity.servo_TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(conf->connectivity.servo_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(conf->connectivity.servo_TIM, ENABLE);

	TIM_Cmd(conf->connectivity.servo_TIM, ENABLE);
err_occured:
	return err;
}

gyro_error ServoInitPeriph(servo_conf* conf)
{
	gyro_error err = NO_ERROR;

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
err_occured:
	return err;
}

gyro_error ServoInit(servo_conf* conf,
		     uint16_t pin,
		     GPIO_TypeDef* port,
		     TIM_TypeDef* tim)
{
	gyro_error err = NO_ERROR;
	if ((conf == NULL) || (port == NULL) || (tim == NULL)) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}

	if ( (err = ServoInitConnectivity(&(conf->connectivity),
		   pin, port, tim)) != NO_ERROR) {
		goto err_occured;
	}
	if ( (err = ServoInitOptions(conf)) != NO_ERROR ) {
		goto err_occured;
	}
	if ( (err = ServoInitPeriph(conf)) != NO_ERROR ) {
		goto err_occured;
	}
	if ( (err = ServoSetDefaultTiming(conf)) != NO_ERROR ) {
		goto err_occured;
	}
err_occured:
	return err;
}