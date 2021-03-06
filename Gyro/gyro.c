#include "gyro.h"

#define BLOCK_SIZE 32

gyro_data_t g_gyro;
int16_t vels[BLOCK_SIZE * 2];
int16_t coefs[BLOCK_SIZE];
int16_t f_states[BLOCK_SIZE*2 -1];

gyro_data_t* get_gyro_data()
{
	return &g_gyro;
}

gyro_error FillFilterCoefs(int16_t* coef_array)
{
	gyro_error err = NO_ERROR;
	int i = 0;

	if (coef_array == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}

	for (i = 0; i < BLOCK_SIZE/2; i++) {
		coef_array[i] = 0;
	}
	for (i = BLOCK_SIZE/2; i< BLOCK_SIZE; i++) {
		coef_array[i] = 2048;
	}
err_occured:
	return err;
}

gyro_error FillGlobalData(gyro_data_t *g_data)
{
	gyro_error err;
	SPI_PIN_conf sck_pin;
	SPI_PIN_conf miso_pin;
	SPI_PIN_conf mosi_pin;
	SPI_PIN_conf cs_pin;
	interrupt_pin_conf int2_pin_conf;

	if ( (err = InitDataBuffer(&(g_data->vel_data), BLOCK_SIZE, vels)) != NO_ERROR) {
		goto err_occured;
	}

	FillFilterCoefs(coefs);
	arm_fir_decimate_init_q15(&(g_data->filter), BLOCK_SIZE, BLOCK_SIZE, coefs, f_states, BLOCK_SIZE);

	g_data->regulator.Kp = 150;
	g_data->regulator.Kd = 0;
	g_data->regulator.Ki = 0;
	arm_pid_init_q15(&(g_data->regulator), 0);

	sck_pin.SPIx_PIN = GPIO_Pin_5;
	sck_pin.SPIx_GPIO_PORT = GPIOA;
	sck_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOA;
	sck_pin.SPIx_SOURCE = GPIO_PinSource5;
	sck_pin.SPIx_AF = GPIO_AF_SPI1;

	mosi_pin.SPIx_PIN = GPIO_Pin_7;
	mosi_pin.SPIx_GPIO_PORT = GPIOA;
	mosi_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOA;
	mosi_pin.SPIx_SOURCE = GPIO_PinSource7;
	mosi_pin.SPIx_AF = GPIO_AF_SPI1;

	miso_pin.SPIx_PIN = GPIO_Pin_6;
	miso_pin.SPIx_GPIO_PORT = GPIOA;
	miso_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOA;
	miso_pin.SPIx_SOURCE = GPIO_PinSource6;
	miso_pin.SPIx_AF = GPIO_AF_SPI1;

	cs_pin.SPIx_PIN = GPIO_Pin_5;
	cs_pin.SPIx_GPIO_PORT = GPIOC;
	cs_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOC;

//For STM32F401C Discovery board only!!!!
//Disable another device that uses SPI1 Interface
//Remove for custom projects!!!!!
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_SetBits(GPIOE, GPIO_Pin_3);
///////////////////////////////////////////////////

	if ( (err = l3g4200dInit(&(g_data->gyroscope),
			SPI1, RCC_APB2Periph_SPI1,
			&sck_pin, &mosi_pin, &miso_pin, &cs_pin)) != NO_ERROR) {
		goto err_occured;
	}

	int2_pin_conf.INTx_pin = GPIO_Pin_1;
	int2_pin_conf.INTx_GPIO_PORT = GPIOB;
//	int2_pin_conf.INTx_pin = GPIO_Pin_0;
//	int2_pin_conf.INTx_GPIO_PORT = GPIOA;
	int2_pin_conf.INTx_GPIO_CLK = RCC_AHB1Periph_GPIOB;
	int2_pin_conf.EXTI_port_source = EXTI_PortSourceGPIOB;
//	int2_pin_conf.INTx_GPIO_CLK = RCC_AHB1Periph_GPIOA;
//	int2_pin_conf.EXTI_port_source = EXTI_PortSourceGPIOA;
	int2_pin_conf.EXTI_pin_source = EXTI_PinSource1;
//	int2_pin_conf.EXTI_pin_source = EXTI_PinSource0;
	int2_pin_conf.EXTI_line = EXTI_Line1;
//	int2_pin_conf.EXTI_line = EXTI_Line0;
	int2_pin_conf.EXTIx_irqn = EXTI1_IRQn;
//	int2_pin_conf.EXTIx_irqn = EXTI0_IRQn;

	if ( (err = l3g4200dSetDataReadyInterrupt(&(g_data->gyroscope), &int2_pin_conf)) != NO_ERROR) {
		goto err_occured;
	}

//TIM3 CH1 PC6
	if( (err = ServoInit(&(g_gyro.servo), GPIO_Pin_6,
			     GPIOC, TIM3, 1)) != NO_ERROR ) {
		goto err_occured;
	}
err_occured:
	return err;
}

int main(void)
{
/*At this stage the microcontroller clock setting is already configured,
  this is done through SystemInit() function which is called from startup
  files (startup_stm32f40xx.s/startup_stm32f427x.s) before to branch to
  application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f4xx.c file
*/
//	gyro_error err;
	int16_t filtered_vel = 0;
	int16_t new_speed = 0;
	int pid_cycles = 0;
	if((FillGlobalData(&g_gyro)) != NO_ERROR) {
		goto err_occured;
	}

	while(1) {
		if (g_gyro.vel_data.block_ready == 1) {
			g_gyro.vel_data.block_ready = 0;
			arm_fir_decimate_q15(&(g_gyro.filter), g_gyro.vel_data.previous_session_buffer_start, &filtered_vel, BLOCK_SIZE);
			new_speed = arm_pid_q15(&(g_gyro.regulator), filtered_vel+15);
			if (new_speed > 350) {
				new_speed = 350;
			} else if (new_speed < -350) {
				new_speed = -350;
			} /*else if ((new_speed > -15) && (new_speed < 15)) {
				new_speed = 0;
				arm_pid_reset_q15(&(g_gyro.regulator));
			}*/
			ServoSetSpeed(&(g_gyro.servo), 1500 + new_speed);
/*			if(pid_cycles++ > 90) {
				arm_pid_reset_q15(&(g_gyro.regulator));
				pid_cycles = 0;
			}
			*/
		}
	}

err_occured:
	while (1) {

	}
}
