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
#define L3G4200D_MAX_FIFO_INDEX 31

typedef uint8_t axis_enable;

typedef enum {
	ACTIVE_SPI = 0,
	ACTIVE_I2C = 1
} l3g4200d_active_bus;

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
	NORMAL	        = 0x02
} l3g4200d_mode;

typedef enum {
	HPM_NORMAL_MODE_RES	= 0x00,
	HPM_REF_SIGNAL		= 0x01,
	HPM_NORMAL_MODE		= 0x02,
	HPM_AUTORESET_INT	= 0x03
} l3g4200d_HPF_mode;

typedef enum {
	HPFCF_0 = 0x00,
	HPFCF_1 = 0x01,
	HPFCF_2 = 0x02,
	HPFCF_3 = 0x03,
	HPFCF_4 = 0x04,
	HPFCF_5 = 0x05,
	HPFCF_6 = 0x06,
	HPFCF_7 = 0x07,
	HPFCF_8 = 0x08,
	HPFCF_9 = 0x09
} l3g4200d_HPFCutOffFreq;

typedef enum {
	FIFO_DISABLE			= 0x05,
	FIFO_BYPASS_MODE		= 0x00,
	FIFO_MODE			= 0x01,
	FIFO_STREAM_MODE		= 0x02,
	FIFO_STREAM_TO_FIFO_MODE	= 0x03,
	FIFO_BYPASS_TO_STREAM_MODE	= 0x04
} l3g4200d_fifo_mode;

typedef enum {
	AXIS_X,
	AXIS_Y,
	AXIS_Z
} l3g4200d_axis;

typedef struct {
	uint8_t ID;
} l3g4200d_WHO_AM_I;

typedef struct {
	uint8_t Xen	: 1;
	uint8_t Yen	: 1;
	uint8_t Zen	: 1;
	uint8_t PD	: 1;
	uint8_t DR_BW	: 4;
} l3g4200d_CTRL_REG1;

typedef struct {
	l3g4200d_HPFCutOffFreq HPCF	: 4;
	l3g4200d_HPF_mode HPM		: 2;
	uint8_t				: 2;
} l3g4200d_CTRL_REG2;

typedef struct {
	uint8_t I2_Empty	: 1;
	uint8_t I2_ORun		: 1;
	uint8_t I2_WTM		: 1;
	uint8_t I2_DRDY		: 1;
	uint8_t PP_OD		: 1;
	uint8_t H_Lactive	: 1;
	uint8_t I1_Boot		: 1;
	uint8_t I1_Int1		: 1;
} l3g4200d_CTRL_REG3;

typedef struct {
	uint8_t SIM	: 1;
	uint8_t ST	: 2;
	uint8_t		: 1;
	uint8_t FS	: 2;
	uint8_t BLE	: 1;
	uint8_t BDU	: 1;
} l3g4200d_CTRL_REG4;

typedef struct {
	uint8_t Out_Sel		: 2;
	uint8_t Int1_Sel	: 2;
	uint8_t HPen		: 1;
	uint8_t 		: 1;
	uint8_t FIFO_EN		: 1;
	uint8_t	BOOT		: 1;
} l3g4200d_CTRL_REG5;

typedef struct {
	uint8_t	ref;
} l3g4200d_REFERENCE;

typedef struct {
	uint8_t	Temp;
} l3g4200d_OUT_TEMP;

typedef struct {
	uint8_t XDA	: 1;
	uint8_t YDA	: 1;
	uint8_t ZDA	: 1;
	uint8_t ZYXDA	: 1;
	uint8_t XOR	: 1;
	uint8_t YOR	: 1;
	uint8_t ZOR	: 1;
	uint8_t	ZYXOR	: 1;
} l3g4200d_STATUS_REG;

typedef struct {
	uint8_t	border;
} l3g4200d_out_axis_border;

typedef struct {
	uint8_t WTM	: 5;
	uint8_t	FM	: 3;
} l3g4200d_FIFO_CTRL_REG;

typedef struct {
	uint8_t	WTM	: 1;
	uint8_t OVRN	: 1;
	uint8_t EMPTY	: 1;
	uint8_t FSS	: 5;
} l3g4200d_FIFO_SRC_REG;

typedef struct {
	uint8_t XLIE	: 1;
	uint8_t XHIE	: 1;
	uint8_t YLIE	: 1;
	uint8_t YHIE	: 1;
	uint8_t ZLIE	: 1;
	uint8_t ZHIE	: 1;
	uint8_t LIR	: 1;
	uint8_t	AND_OR	: 1;
} l3g4200d_INT1_CFG;

typedef struct {
	uint8_t XL	: 1;
	uint8_t XH	: 1;
	uint8_t YL	: 1;
	uint8_t YH	: 1;
	uint8_t ZL	: 1;
	uint8_t ZH	: 1;
	uint8_t IA	: 1;
	uint8_t 	: 1;
} l3g4200d_INT1_SRC;

typedef struct {
	uint16_t level	: 15;
	uint16_t 	: 1;
} l3g4200d_Int_Threshold_Level;

typedef struct {
	uint8_t Duration	: 7;
	uint8_t WAIT		: 1;
} l3g4200d_INT1_DURATION;

typedef struct {
	uint16_t SPIx_PIN;
	GPIO_TypeDef *SPIx_GPIO_PORT;
	uint32_t SPIx_GPIO_CLK;
	uint8_t SPIx_SOURCE;
	uint8_t SPIx_AF;
} SPI_PIN_conf;

typedef struct {
	uint16_t INTx_pin;
	GPIO_TypeDef* INTx_GPIO_PORT;
	uint32_t INTx_GPIO_CLK;

	uint8_t EXTI_port_source;
	uint8_t EXTI_pin_source;

	uint32_t EXTI_line;
	uint32_t EXTIx_irqn;
} interrupt_pin_conf;

typedef struct {
	l3g4200d_active_bus active_bus;
	SPI_TypeDef *SPIx;	//SPI1, SPI2 etc.
	uint32_t SPIx_CLK;	//RCC_APB1Periph_SPIx

	SPI_PIN_conf sck_pin;
	SPI_PIN_conf miso_pin;
	SPI_PIN_conf mosi_pin;
	SPI_PIN_conf cs_pin;

	interrupt_pin_conf int2_pin;
	
	struct_init_status init_status;
} l3g4200d_connectivity_conf;

typedef struct {
	struct_init_status init_status;
	uint8_t device_id;
	l3g4200d_connectivity_conf connectivity;
} l3g4200d_conf;

uint32_t l3g4200dTimeoutCallback();
gyro_error l3g4200dInit(l3g4200d_conf*, SPI_TypeDef*, uint32_t,
			SPI_PIN_conf*, SPI_PIN_conf*,
			SPI_PIN_conf*, SPI_PIN_conf*);
gyro_error l3g4200dSetDataReadyInterrupt(l3g4200d_conf* conf, interrupt_pin_conf* intx_pin_conf);

gyro_error l3g4200dReadAngularVelocity(l3g4200d_conf*, l3g4200d_axis,
					int16_t*);
void l3g4200dReadAngularVelocity_Unsafe_DataReady(l3g4200d_conf* conf, l3g4200d_axis axis,
					int16_t* velocity);
void l3g4200dReadAngularVelocity_Unsafe(l3g4200d_conf* conf, l3g4200d_axis axis,
					int16_t* velocity);


gyro_error l3g4200dReadAngularVelocityBulk(l3g4200d_conf* conf,
					int16_t* velocities);
void l3g4200dReadAngularVelocityBulk_Unsafe_DataReady(l3g4200d_conf* conf,
					int16_t* velocities);
void l3g4200dReadAngularVelocityBulk_Unsafe(l3g4200d_conf* conf,
					int16_t* velocities);


gyro_error l3g4200dRead(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_read, l3g4200d_conf *conf);
void l3g4200dRead_Unsafe(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_read, l3g4200d_conf *conf);
gyro_error l3g4200dWrite(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_write, l3g4200d_conf *conf);
void l3g4200dWrite_Unsafe(uint8_t *buffer, uint8_t addr,
			     uint16_t bytes_to_write, l3g4200d_conf *conf);

void l3g4200dIsNewDataAvailable_Unsafe(l3g4200d_conf* conf, int* available);
gyro_error l3g4200dIsNewDataAvailable(l3g4200d_conf* conf, int* available);



#endif
