#include "L3G4200D.h"

l3g4200d_error l3g4200d_set_active_bus(l3g4200d_active_bus bus, l3g4200d_conf *conf)
{
	if(conf == NULL) {
		return ERROR_NULL_POINTER;
	} else if(bus > ACTIVE_I2C/* || bus < ACTIVE_SPI*/) {
		return ERROR_VALUE_NOT_IN_RANGE;
	} else {
		conf->connectivity.active_bus = bus;
		return NO_ERROR;
	}	
}

l3g4200d_error l3g4200d_get_active_bus(l3g4200d_conf *conf,
					l3g4200d_active_bus *bus)
{
	if(conf == NULL || bus == NULL) {
		return ERROR_NULL_POINTER;
	} else {
		*bus = conf->connectivity.active_bus;
		return NO_ERROR;
	}
}

l3g4200d_error l3g4200d_check_device_id(l3g4200d_conf *conf)
{
	if (conf == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conf->device_id != DEFAULT_DEVICE_ID) {
		return ERROR_DEVICE_ID_NOT_VALID;
	} else {
		return NO_ERROR;
	}
}

l3g4200d_init_status l3g4200d_is_initialized(l3g4200d_conf *conf)
{
	if(conf == NULL) {
		return STRUCT_NOT_INITIALIZED;
	} else {
		return conf->init_status;
	}
}

l3g4200d_error l3g4200d_init_connectivity(l3g4200d_connectivity_conf *conn,
SPI_TypeDef *SPIx, uint32_t SPIx_CLK,
uint16_t SCK_PIN, GPIO_TypeDef *SCK_GPIO_PORT, uint32_t SCK_GPIO_CLK, uint8_t SCK_SOURCE, uint8_t SCK_AF,
uint16_t MISO_PIN, GPIO_TypeDef *MISO_GPIO_PORT, uint32_t MISO_GPIO_CLK, uint8_t MISO_SOURCE, uint8_t MISO_AF,
uint16_t MOSI_PIN, GPIO_TypeDef *MOSI_GPIO_PORT, uint32_t MOSI_GPIO_CLK, uint8_t MOSI_SOURCE, uint8_t MOSI_AF,
uint16_t CS_PIN, GPIO_TypeDef *CS_GPIO_PORT, uint32_t CS_GPIO_CLK)
{
	if (conn == NULL) {
		return ERROR_NULL_POINTER;
	} else {
		conn->active_bus = ACTIVE_SPI;
		
		conn->SPIx = SPIx;
		conn->SPIx_CLK = SPIx_CLK;

		conn->sck_pin.SPIx_PIN = SCK_PIN;
		conn->sck_pin.SPIx_GPIO_PORT = SCK_GPIO_PORT;
		conn->sck_pin.SPIx_GPIO_CLK = SCK_GPIO_CLK;
		conn->sck_pin.SPIx_SOURCE = SCK_SOURCE;
		conn->sck_pin.SPIx_AF = SCK_AF;

		conn->miso_pin.SPIx_PIN = MISO_PIN;
		conn->miso_pin.SPIx_GPIO_PORT = MISO_GPIO_PORT;
		conn->miso_pin.SPIx_GPIO_CLK = MISO_GPIO_CLK;
		conn->miso_pin.SPIx_SOURCE = MISO_SOURCE;
		conn->miso_pin.SPIx_AF = MISO_AF;

		conn->mosi_pin.SPIx_PIN = MOSI_PIN;
		conn->mosi_pin.SPIx_GPIO_PORT = MOSI_GPIO_PORT;
		conn->mosi_pin.SPIx_GPIO_CLK = MOSI_GPIO_CLK;
		conn->mosi_pin.SPIx_SOURCE = MOSI_SOURCE;
		conn->mosi_pin.SPIx_AF = MOSI_AF;

		conn->cs_pin.SPIx_PIN = CS_PIN;
		conn->cs_pin.SPIx_GPIO_PORT = CS_GPIO_PORT;
		conn->cs_pin.SPIx_GPIO_CLK = CS_GPIO_CLK;

		conn->init_status = STRUCT_INITIALIZED;
	}
	return NO_ERROR;
}

l3g4200d_error l3g4200d_init_periph(l3g4200d_connectivity_conf *conn)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	if(conn == NULL) {
		return ERROR_NULL_POINTER;
	} else if(conn->init_status == DEVICE_NOT_INITIALIZED) {
		return ERROR_DEVICE_NOT_INITIALIZED;
	} else {
		RCC_APB1PeriphClockCmd( conn->SPIx_CLK, ENABLE);
		RCC_AHB1PeriphClockCmd( conn->cs_pin.SPIx_GPIO_CLK |
					conn->mosi_pin.SPIx_GPIO_CLK |
					conn->miso_pin.SPIx_GPIO_CLK |
					conn->cs_pin.SPIx_GPIO_CLK, ENABLE);
		GPIO_PinAFConfig(conn->sck_pin.SPIx_GPIO_PORT,
				 conn->sck_pin.SPIx_SOURCE,
				 conn->sck_pin.SPIx_AF);
		GPIO_PinAFConfig(conn->miso_pin.SPIx_GPIO_PORT,
				 conn->miso_pin.SPIx_SOURCE,
				 conn->miso_pin.SPIx_AF);
		GPIO_PinAFConfig(conn->mosi_pin.SPIx_GPIO_PORT,
				 conn->mosi_pin.SPIx_SOURCE,
				 conn->mosi_pin.SPIx_AF);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_InitStructure.GPIO_Pin = conn->sck_pin.SPIx_PIN;
		GPIO_Init(conn->sck_pin.SPIx_GPIO_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  conn->mosi_pin.SPIx_PIN;
		GPIO_Init(conn->mosi_pin.SPIx_GPIO_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = conn->miso_pin.SPIx_PIN;
		GPIO_Init(conn->miso_pin.SPIx_GPIO_PORT, &GPIO_InitStructure);

		SPI_I2S_DeInit(conn->SPIx);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_Init(conn->SPIx, &SPI_InitStructure);

		SPI_Cmd(conn->SPIx, ENABLE);

		GPIO_InitStructure.GPIO_Pin = conn->cs_pin.SPIx_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(conn->cs_pin.SPIx_GPIO_PORT, &GPIO_InitStructure);

		GPIO_SetBits(conn->cs_pin.SPIx_GPIO_PORT, conn->cs_pin.SPIx_PIN);

		return NO_ERROR;
	}
}

l3g4200d_error l3g4200d_init(l3g4200d_conf *conf,
SPI_TypeDef *SPIx, uint32_t SPIx_CLK,
uint16_t SCK_PIN, GPIO_TypeDef *SCK_GPIO_PORT, uint32_t SCK_GPIO_CLK, uint8_t SCK_SOURCE, uint8_t SCK_AF,
uint16_t MISO_PIN, GPIO_TypeDef *MISO_GPIO_PORT, uint32_t MISO_GPIO_CLK, uint8_t MISO_SOURCE, uint8_t MISO_AF,
uint16_t MOSI_PIN, GPIO_TypeDef *MOSI_GPIO_PORT, uint32_t MOSI_GPIO_CLK, uint8_t MOSI_SOURCE, uint8_t MOSI_AF,
uint16_t CS_PIN, GPIO_TypeDef *CS_GPIO_PORT, uint32_t CS_GPIO_CLK)
{
	if (conf == NULL) {
		return ERROR_NULL_POINTER;
	} else {
		l3g4200d_init_connectivity(&(conf->connectivity), SPIx, SPIx_CLK,
			SCK_PIN, SCK_GPIO_PORT, SCK_GPIO_CLK, SCK_SOURCE, SCK_AF,
			MISO_PIN, MISO_GPIO_PORT, MISO_GPIO_CLK, MISO_SOURCE, MISO_AF,
			MOSI_PIN, MOSI_GPIO_PORT, MOSI_GPIO_CLK, MOSI_SOURCE, MOSI_AF,
			CS_PIN, CS_GPIO_PORT, CS_GPIO_CLK);
		l3g4200d_init_periph(&(conf->connectivity));
	}
	return NO_ERROR;
}

l3g4200d_error l3g4200d_read(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_read, l3g4200d_conf *conf)
{
	if (buffer == NULL || conf == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conf->connectivity.init_status == STRUCT_NOT_INITIALIZED) {
		return ERROR_DEVICE_NOT_INITIALIZED;
	} else if (bytes_to_read < 1) {
		return ERROR_VALUE_NOT_IN_RANGE;
	} else {
		
	}
}
