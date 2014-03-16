#include "L3G4200D.h"

#define READWRITE_CMD			((uint8_t)0x80) 
#define MULTIPLEBYTE_CMD		((uint8_t)0x40)
#define DUMMY_BYTE			((uint8_t)0x00)

gyro_error l3g4200dSetActiveBus(l3g4200d_active_bus bus,
					l3g4200d_conf *conf)
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

gyro_error l3g4200dGetActiveBus(l3g4200d_conf *conf,
					l3g4200d_active_bus *bus)
{
	if(conf == NULL || bus == NULL) {
		return ERROR_NULL_POINTER;
	} else {
		*bus = conf->connectivity.active_bus;
		return NO_ERROR;
	}
}

gyro_error l3g4200dCheckDeviceId(l3g4200d_conf *conf)
{
	if (conf == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conf->device_id != DEFAULT_DEVICE_ID) {
		return ERROR_DEVICE_ID_NOT_VALID;
	} else {
		return NO_ERROR;
	}
}

l3g4200d_init_status l3g4200dIsInitialized(l3g4200d_conf *conf)
{
	if(conf == NULL) {
		return STRUCT_NOT_INITIALIZED;
	} else {
		return conf->init_status;
	}
}

gyro_error l3g4200dInitConnectivity(l3g4200d_connectivity_conf *conn,
					SPI_TypeDef *SPIx, uint32_t SPIx_CLK,
					SPI_PIN_conf *pin_sck,
					SPI_PIN_conf *pin_mosi,
					SPI_PIN_conf *pin_miso,
					SPI_PIN_conf *pin_cs)
{
	if (conn == NULL || SPIx == NULL || pin_sck == NULL ||
	    pin_mosi == NULL || pin_miso == NULL || pin_cs == NULL) {
		return ERROR_NULL_POINTER;
	} else {
		conn->active_bus = ACTIVE_SPI;
		
		conn->SPIx = SPIx;
		conn->SPIx_CLK = SPIx_CLK;

		conn->sck_pin.SPIx_PIN = pin_sck->SPIx_PIN;
		conn->sck_pin.SPIx_GPIO_PORT = pin_sck->SPIx_GPIO_PORT;
		conn->sck_pin.SPIx_GPIO_CLK = pin_sck->SPIx_GPIO_CLK;
		conn->sck_pin.SPIx_SOURCE = pin_sck->SPIx_SOURCE;
		conn->sck_pin.SPIx_AF = pin_sck->SPIx_AF;

		conn->miso_pin.SPIx_PIN = pin_miso->SPIx_PIN;
		conn->miso_pin.SPIx_GPIO_PORT = pin_miso->SPIx_GPIO_PORT;
		conn->miso_pin.SPIx_GPIO_CLK = pin_miso->SPIx_GPIO_CLK;
		conn->miso_pin.SPIx_SOURCE = pin_miso->SPIx_SOURCE;
		conn->miso_pin.SPIx_AF = pin_miso->SPIx_AF;

		conn->mosi_pin.SPIx_PIN = pin_mosi->SPIx_PIN;
		conn->mosi_pin.SPIx_GPIO_PORT = pin_mosi->SPIx_GPIO_PORT;
		conn->mosi_pin.SPIx_GPIO_CLK = pin_mosi->SPIx_GPIO_CLK;
		conn->mosi_pin.SPIx_SOURCE = pin_mosi->SPIx_SOURCE;
		conn->mosi_pin.SPIx_AF = pin_mosi->SPIx_AF;

		conn->cs_pin.SPIx_PIN = pin_cs->SPIx_PIN;
		conn->cs_pin.SPIx_GPIO_PORT = pin_cs->SPIx_GPIO_PORT;
		conn->cs_pin.SPIx_GPIO_CLK = pin_cs->SPIx_GPIO_CLK;

		conn->init_status = STRUCT_INITIALIZED;
	}
	return NO_ERROR;
}

gyro_error l3g4200dInitPeriph(l3g4200d_connectivity_conf *conn)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	if(conn == NULL) {
		return ERROR_NULL_POINTER;
	} else if(conn->init_status == STRUCT_NOT_INITIALIZED) {
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

gyro_error l3g4200dInitDefaultSettings(l3g4200d_settings *set)
{
	if (set == NULL) {
		return ERROR_NULL_POINTER;
	}

	set->ODR = ODR_100Hz_BW_12_5;
	set->axis_state = (l3g4200d_axis_state)(X_ENABLE | Y_ENABLE | Z_ENABLE);
	set->fullscale_state = FULLSCALE_250;
	set->mode = NORMAL;
	set->fifo_mode = FIFO_MODE;
	return NO_ERROR;
}

gyro_error l3g4200dInit(l3g4200d_conf *conf,
			SPI_TypeDef *SPIx, uint32_t SPIx_CLK,
			SPI_PIN_conf *pin_sck,
			SPI_PIN_conf *pin_mosi,
			SPI_PIN_conf *pin_miso,
			SPI_PIN_conf *pin_cs)
{
	gyro_error err;
	if (conf == NULL || pin_sck == NULL || pin_mosi == NULL ||
	    pin_miso == NULL || pin_cs == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
	if ((err = l3g4200dInitConnectivity(&(conf->connectivity), SPIx,
	SPIx_CLK, pin_sck, pin_mosi, pin_miso, pin_cs)) != NO_ERROR) {
		goto err_occured;
	}
	if ((err = l3g4200dInitPeriph(&(conf->connectivity))) != NO_ERROR) {
		goto err_occured;
	}
	if ((err = l3g4200dInitDefaultSettings(&(conf->settings))) != NO_ERROR) {
		goto err_occured;
	}
err_occured:
	return err;
}

gyro_error l3g4200dCsLow(l3g4200d_connectivity_conf *conn)
{
	if(conn == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conn->init_status == STRUCT_NOT_INITIALIZED) {
		return ERROR_DEVICE_NOT_INITIALIZED;
	} else {
		GPIO_ResetBits(conn->cs_pin.SPIx_GPIO_PORT,
				conn->cs_pin.SPIx_PIN);
		return NO_ERROR;
	}
}

gyro_error l3g4200dCsHigh(l3g4200d_connectivity_conf *conn)
{
	if(conn == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conn->init_status == STRUCT_NOT_INITIALIZED) {
		return ERROR_DEVICE_NOT_INITIALIZED;
	} else {
		GPIO_SetBits(conn->cs_pin.SPIx_GPIO_PORT,
				conn->cs_pin.SPIx_PIN);
		return NO_ERROR;
	}
}

uint8_t l3g4200dSendByte(l3g4200d_connectivity_conf *conn, uint8_t msg)
{
	uint32_t timeout_counter = L3G4200D_TIMEOUT_COUNTER;
	if (conn == NULL) {
		return 0;
	} else {
		while(SPI_I2S_GetFlagStatus(conn->SPIx, SPI_I2S_FLAG_TXE) == RESET) {
			if((timeout_counter--) == 0) {
				return l3g4200dTimeoutCallback();
			}
		}
		SPI_I2S_SendData(conn->SPIx, (uint16_t)msg);
		
		timeout_counter = L3G4200D_TIMEOUT_COUNTER;
		while (SPI_I2S_GetFlagStatus(conn->SPIx, SPI_I2S_FLAG_RXNE) == RESET) {
			if((timeout_counter--) == 0) {
				return l3g4200dTimeoutCallback();
			}
		}
		return (uint8_t)SPI_I2S_ReceiveData(conn->SPIx);
	}
}

gyro_error l3g4200dRead(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_read, l3g4200d_conf *conf)
{
	uint8_t address = 0;	
	gyro_error ret_err = NO_ERROR;

	if (buffer == NULL || conf == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conf->connectivity.init_status == STRUCT_NOT_INITIALIZED) {
		return ERROR_DEVICE_NOT_INITIALIZED;
	} else if (bytes_to_read < 1) {
		return ERROR_VALUE_NOT_IN_RANGE;
	} else {
		if (bytes_to_read > 1) {
			address = (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD | addr);
		} else {
			address = (uint8_t)(READWRITE_CMD | addr);
		}
		ret_err = l3g4200dCsLow(&(conf->connectivity));
		if (ret_err != NO_ERROR)
			goto read_err;
		l3g4200dSendByte(&(conf->connectivity), address);

		while(bytes_to_read > 0) {
/* Send dummy byte (0x00) to generate the SPI clock to L3G4200D (Slave device) */
			*buffer = l3g4200dSendByte(&(conf->connectivity), DUMMY_BYTE);
			bytes_to_read--;
			buffer++;
		}

		ret_err = l3g4200dCsHigh(&(conf->connectivity));
		if (ret_err != NO_ERROR) {
			goto read_err;
		}
	}
	return NO_ERROR;
read_err:
	return ret_err;
}

gyro_error l3g4200dWrite(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_write, l3g4200d_conf *conf)
{
	uint8_t address = 0;	
	gyro_error ret_err = NO_ERROR;

	if (buffer == NULL || conf == NULL) {
		return ERROR_NULL_POINTER;
	} else if (conf->connectivity.init_status == STRUCT_NOT_INITIALIZED) {
		return ERROR_DEVICE_NOT_INITIALIZED;
	} else if (bytes_to_write < 1) {
		return ERROR_VALUE_NOT_IN_RANGE;
	} else {
		if (bytes_to_write > 1) {
			address = (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD | addr);
		} else {
			address = (uint8_t)(READWRITE_CMD | addr);
		}
		ret_err = l3g4200dCsLow(&(conf->connectivity));
		if (ret_err != NO_ERROR)
			goto write_err;
		l3g4200dSendByte(&(conf->connectivity), address);
		while (bytes_to_write >= 1) {
			l3g4200dSendByte(&(conf->connectivity), *buffer);
			buffer++;
			bytes_to_write--;
		}

		ret_err = l3g4200dCsHigh(&(conf->connectivity));
		if(ret_err != NO_ERROR) {
			goto write_err;
		}
	}
	return NO_ERROR;
write_err:
	return ret_err;
}

/*TODO: make device reset procedure*/
uint32_t l3g4200dTimeoutCallback()
{
	while(1) {

	}
}
