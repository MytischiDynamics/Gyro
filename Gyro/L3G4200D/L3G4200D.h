#ifndef L3G4200D_H
#define L3G4200D_H

#include "stm32f4xx.h"
#include "gyro_error.h"

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

typedef uint8_t axis_enable;

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

typedef enum {
	X_ENABLE	= 0x04,
	X_DISABLE	= 0x00,
	Y_ENABLE	= 0x02,
	Y_DISABLE	= 0x00,
	Z_ENABLE	= 0x01,
	Z_DISABLE	= 0x00
} l3g4200d_axis_state;

typedef enum {
	FULLSCALE_250	= 0x00,
	FULLSCALE_500	= 0x01,
	FULLSCALE_2000	= 0x02
} l3g4200d_fullscale_state;

typedef enum {
	POWER_DOWN	= 0x00,
	SLEEP		= 0x01,
	NORMAL	= 0x02
} l3g4200d_mode;

typedef enum {
	FIFO_DISABLE			= 0x05,
	FIFO_BYPASS_MODE		= 0x00,
	FIFO_MODE			= 0x01,
	FIFO_STREAM_MODE		= 0x02,
	FIFO_STREAM_TO_FIFO_MODE	= 0x03,
	FIFO_BYPASS_TO_STREAM_MODE	= 0x04
} l3g4200f_fifo_mode;

typedef struct {
	uint8_t ID;
} l3g4200d_WHO_AM_I;

typedef struct {
	uint8_t DR_BW	: 4;
	uint8_t PD	: 1;
	uint8_t Zen	: 1;
	uint8_t Yen	: 1;
	uint8_t Xen	: 1;
} l3g4200d_CTRL_REG1;

typedef struct {
	uint8_t		: 2;
	uint8_t HPM	: 2;
	uint8_t HPCF	: 4;
} l3g4200d_CTRL_REG2;

typedef struct {
	uint8_t I1_Int1		: 1;
	uint8_t I1_Boot		: 1;
	uint8_t H_Lactive	: 1;
	uint8_t PP_OD		: 1;
	uint8_t I2_DRDY		: 1;
	uint8_t I2_WTM		: 1;
	uint8_t I2_ORun		: 1;
	uint8_t I2_Empty	: 1;
} l3g4200d_CTRL_REG3;

typedef struct {
	uint8_t BDU	: 1;
	uint8_t BLE	: 1;
	uint8_t FS	: 2;
	uint8_t		: 1;
	uint8_t ST	: 2;
	uint8_t SIM	: 1;
} l3g4200d_CTRL_REG4;

typedef struct {
	uint8_t	BOOT		: 1;
	uint8_t FIFO_EN		: 1;
	uint8_t 		: 1;
	uint8_t HPen		: 1;
	uint8_t Int1_Sel	: 2;
	uint8_t Out_Sel		: 2;
} l3g4200d_CTRL_REG5;

typedef struct {
	uint8_t	ref;
} l3g4200d_REFERENCE;

typedef struct {
	uint8_t	Temp;
} l3g4200d_OUT_TEMP;

typedef struct {
	uint8_t	ZYXOR	: 1;
	uint8_t ZOR	: 1;
	uint8_t YOR	: 1;
	uint8_t XOR	: 1;
	uint8_t ZYXDA	: 1;
	uint8_t ZDA	: 1;
	uint8_t YDA	: 1;
	uint8_t XDA	: 1;
} l3g4200d_STATUS_REG;

typedef struct {
	uint8_t	border;
} l3g4200d_out_axis_border;

typedef struct {
	uint8_t	FM	: 3;
	uint8_t WTM	: 5;
} l3g4200d_FIFO_CTRL_REG;

typedef struct {
	uint8_t	WTM	: 1;
	uint8_t OVRN	: 1;
	uint8_t EMPTY	: 1;
	uint8_t FSS	: 5;
} l3g4200d_FIFO_SRC_REG;

typedef struct {
	uint8_t	AND_OR	: 1;
	uint8_t LIR	: 1;
	uint8_t ZHIE	: 1;
	uint8_t ZLIE	: 1;
	uint8_t YHIE	: 1;
	uint8_t YLIE	: 1;
	uint8_t XHIE	: 1;
	uint8_t XLIE	: 1;
} l3g4200d_INT1_CFG;

typedef struct {
	uint8_t 	: 1;
	uint8_t IA	: 1;
	uint8_t ZH	: 1;
	uint8_t ZL	: 1;
	uint8_t YH	: 1;
	uint8_t YL	: 1;
	uint8_t XH	: 1;
	uint8_t XL	: 1;
} l3g4200d_INT1_SRC;

typedef struct {
	uint16_t 	: 1;
	uint16_t level	: 15;
} l3g4200d_Int_Threshold_Level;

typedef struct {
	uint8_t WAIT		: 1;
	uint8_t Duration	: 7;
} l3g4200d_INT1_DURATION;

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
/*
typedef struct {
	l3g4200d_output_data_rate ODR;
	l3g4200d_axis_state axis_state;
	l3g4200d_fullscale_state fullscale_state;
	l3g4200d_mode mode;
	l3g4200f_fifo_mode fifo_mode;
} l3g4200d_settings;
*/
typedef struct {
	l3g4200d_init_status init_status;
	uint8_t device_id;
	l3g4200d_connectivity_conf connectivity;
//	l3g4200d_settings settings;
} l3g4200d_conf;

uint32_t l3g4200dTimeoutCallback();
gyro_error l3g4200dInit(l3g4200d_conf*, SPI_TypeDef*, uint32_t,
			SPI_PIN_conf*, SPI_PIN_conf*,
			SPI_PIN_conf*, SPI_PIN_conf*);

#endif
