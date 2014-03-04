#ifndef L3G4200D_H
#define L3G4200D_H

#include "stm32f4xx.h"

#define NULL (void*)0

#define WHO_AM_I	0x0F
#define CTRL_REG1	0x20
#define CTRL_REG2	0x21
#define CTRL_REG3	0x22
#define CTRL_REG4	0x23
#define CTRL_REG5	0x24
#define REFERENCE	0x25
#define OUT_TEMP	0x26
#define STATUS_REG	0x27
#define OUT_X_L		0x28
#define OUT_X_H		0x29 
#define OUT_Y_L		0x2A
#define OUT_Y_H		0X2B
#define OUT_Z_L		0x2C
#define OUT_Z_H		0x2D
#define FIFO_CTRL_REG	0x2E
#define FIFO_SRC_REG	0x2F
#define INT1_CFG	0x30
#define INT1_SRC	0x31
#define INT1_TSH_XH	0x32
#define INT1_TSH_XL	0x33
#define INT1_TSH_YH	0x34
#define INT1_TSH_YL	0x35
#define INT1_TSH_ZH	0x36
#define INT1_TSH_ZL	0x37
#define INT1_DURATION	0x38

#define DEFAULT_DEVICE_ID 0xD3
#define L3G4200D_TIMEOUT_COUNTER ((uint32_t)0x1000)

typedef enum {
	NO_ERROR = 0,
	ERROR_NULL_POINTER,
	ERROR_VALUE_NOT_IN_RANGE,
	ERROR_DEVICE_NOT_INITIALIZED,
	ERROR_DEVICE_ID_NOT_VALID,
	ERROR_NO_DATA_RECEIVED,

	EROR_LAST_ERROR
} l3g4200d_error;

typedef enum {
	ACTIVE_SPI = 0,
	ACTIVE_I2C = 1
} l3g4200d_active_bus;

typedef enum {
	STRUCT_NOT_INITIALIZED = 0,
	STRUCT_INITIALIZED = 1
} l3g4200d_init_status;

typedef enum {  
	ODR_100Hz_BW_12_5	= 0x00,
	ODR_100Hz_BW_25		= 0x01,		
	ODR_200Hz_BW_12_5	= 0x04,
	ODR_200Hz_BW_25		= 0x05,
	ODR_200Hz_BW_50		= 0x06,
	ODR_200Hz_BW_70		= 0x07,	
	ODR_400Hz_BW_20		= 0x08,
	ODR_400Hz_BW_25		= 0x09,
	ODR_400Hz_BW_50		= 0x0A,
	ODR_400Hz_BW_110	= 0x0B,	
	ODR_800Hz_BW_30		= 0x0C,
	ODR_800Hz_BW_35		= 0x0D,
	ODR_800Hz_BW_50		= 0x0E,
	ODR_800Hz_BW_110	= 0x0F
} l3g4200d_output_data_rate;

typedef struct {
	uint16_t SPIx_PIN;
	GPIO_TypeDef *SPIx_GPIO_PORT;
	uint32_t SPIx_GPIO_CLK;
	uint8_t SPIx_SOURCE;
	uint8_t SPIx_AF;
} SPI_PIN_conf;

typedef struct {
	l3g4200d_active_bus active_bus;
	SPI_TypeDef *SPIx;	//SPI1, SPI2 etc.
	uint32_t SPIx_CLK;	//RCC_APB1Periph_SPIx

	SPI_PIN_conf sck_pin;
	SPI_PIN_conf miso_pin;
	SPI_PIN_conf mosi_pin;
	SPI_PIN_conf cs_pin;
	
	l3g4200d_init_status init_status;
} l3g4200d_connectivity_conf;

typedef struct {
	l3g4200d_init_status init_status;
	char device_id;
	l3g4200d_connectivity_conf connectivity;
	l3g4200d_output_data_rate data_rate;
	
} l3g4200d_conf;

uint32_t l3g4200dTimeoutCallback();

#endif
