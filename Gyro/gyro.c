#include "gyro.h"

gyro_data_t g_gyro;

gyro_error FillGlobalData(gyro_data_t *g_data)
{
	gyro_error err;
	SPI_PIN_conf sck_pin;
	SPI_PIN_conf miso_pin;
	SPI_PIN_conf mosi_pin;
	SPI_PIN_conf cs_pin;
	
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
	
/*	if ( (err = l3g4200dInit(&(g_data->gyroscope),
			SPI1, RCC_APB2Periph_SPI1,
			&sck_pin, &mosi_pin, &miso_pin, &cs_pin)) != NO_ERROR) {
		goto err_occured;
	}*/
//TIM3 CH1 PC6
	if( (err = ServoInit(&(g_gyro.servo), GPIO_Pin_6,
			     GPIOC, TIM3, 1)) != NO_ERROR ) {
		goto err_occured;
	}
err_occured:
	return err;
}
/*
void __iar_program_start()
{
}
*/
int main(void)
{
/*At this stage the microcontroller clock setting is already configured,
  this is done through SystemInit() function which is called from startup
  files (startup_stm32f40xx.s/startup_stm32f427x.s) before to branch to
  application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f4xx.c file
*/
	FillGlobalData(&g_gyro);

	ServoSetSpeed(&(g_gyro.servo), 1100);
	ServoSetSpeed(&(g_gyro.servo), 1150);
	ServoSetSpeed(&(g_gyro.servo), 1200);
	ServoSetSpeed(&(g_gyro.servo), 1250);
	ServoSetSpeed(&(g_gyro.servo), 1300);
	ServoSetSpeed(&(g_gyro.servo), 1350);
	ServoSetSpeed(&(g_gyro.servo), 1400);
	ServoSetSpeed(&(g_gyro.servo), 1450);
	ServoSetSpeed(&(g_gyro.servo), 1500);
	ServoSetSpeed(&(g_gyro.servo), 1550);
	ServoSetSpeed(&(g_gyro.servo), 1600);
	ServoSetSpeed(&(g_gyro.servo), 1650);
	ServoSetSpeed(&(g_gyro.servo), 1700);
	ServoSetSpeed(&(g_gyro.servo), 1750);
	ServoSetSpeed(&(g_gyro.servo), 1800);

	while(1) {

	}
}
