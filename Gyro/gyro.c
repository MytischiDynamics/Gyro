#include "gyro.h"

gyro_data_t g_gyro;

gyro_data_t* get_gyro_data()
{
	return &g_gyro;
}

gyro_error FillGlobalData(gyro_data_t *g_data)
{
	gyro_error err;
	SPI_PIN_conf sck_pin;
	SPI_PIN_conf miso_pin;
	SPI_PIN_conf mosi_pin;
	SPI_PIN_conf cs_pin;
	interrupt_pin_conf int2_pin_conf;
	
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
	int2_pin_conf.INTx_GPIO_CLK = RCC_AHB1Periph_GPIOB;
	int2_pin_conf.EXTI_port_source = EXTI_PortSourceGPIOB;
	int2_pin_conf.EXTI_pin_source = EXTI_PinSource1;
	int2_pin_conf.EXTI_line = EXTI_Line1;
	int2_pin_conf.EXTIx_irqn = EXTI1_IRQn;

	if ( (err = l3g4200dSetDataReadyInterrupt(&(g_data->gyroscope), &int2_pin_conf)) != NO_ERROR) {
		goto err_occured;
	}

//TIM3 CH1 PC6
/*	if( (err = ServoInit(&(g_gyro.servo), GPIO_Pin_6,
			     GPIOC, TIM3, 1)) != NO_ERROR ) {
		goto err_occured;
	}*/
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
	if((FillGlobalData(&g_gyro)) != NO_ERROR) {
		goto err_occured;
	}
	g_gyro.vels_count = 0;

	while(1) {

	}

err_occured:
	while (1) {

	}
}
