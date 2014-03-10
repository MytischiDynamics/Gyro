#include "gyro.h"

gyro_data g_gyro;

gyro_error FillGlobalData(gyro_data *g_data)
{
	gyro error err;
	SPI_PIN_conf sck_pin;
	SPI_PIN_conf miso_pin;
	SPI_PIN_conf mosi_pin;
	SPI_PIN_conf cs_pin;
	
	sck_pin.SPIx_PIN = GPIO_Pin_13;
	sck_pin.SPIx_GPIO_PORT = GPIOB;
	sck_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOB;
	sck_pin.SPIx_SOURCE = GPIO_PinSource13;
	sck_pin.SPIx_AF = GPIO_AF_SPI2;

	mosi_pin.SPIx_PIN = GPIO_Pin_15;
	mosi_pin.SPIx_GPIO_PORT = GPIOB;
	mosi_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOB;
	mosi_pin.SPIx_SOURCE = GPIO_PinSource15;
	mosi_pin.SPIx_AF = GPIO_AF_SPI2;

	miso_pin.SPIx_PIN = GPIO_Pin_14;
	miso_pin.SPIx_GPIO_PORT = GPIOB;
	miso_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOB;
	miso_pin.SPIx_SOURCE = GPIO_PinSource14;
	miso_pin.SPIx_AF = GPIO_AF_SPI2;

	sck_pin.SPIx_PIN = GPIO_Pin_12;
	sck_pin.SPIx_GPIO_PORT = GPIOB;
	sck_pin.SPIx_GPIO_CLK = RCC_AHB1Periph_GPIOB;
	
	err = l3g4200dInit(&(g_data->gyroscope.connectivity),
			   SPI2, RCC_APB1Periph_SPI2,
			   &sck_pin, &mosi_pin, &miso_pin, &cs_pin);
	if (err != NO_ERROR) {
		return err;
	}
	
	return NO_ERROR;
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

	while (1) {

	}
}
