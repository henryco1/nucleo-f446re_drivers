/*
 * stm32f446xx_spi_driver.h
 *
 *      Author: henryco1
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/***************************************
 * Configuration Structure for SPI pins
 ***************************************/
typedef struct {
	uint8_t SPI_DeviceMode;						// see @SPI_DEVICE_MODES
	uint8_t SPI_BusConfig;						// see @SPI_BUS_CONFIGURATION
	uint8_t SPI_SerialClkSpeed;					// see @SPI_SERIAL_CLOCK_SPEED
	uint8_t SPI_DataFrameFormat;				// see @SPI_DATA_FRAME_FORMAT
	uint8_t SPI_CPOL;							// see @SPI_CLOCK_POLARITY
	uint8_t SPI_CPHA;							// see @SPI_CLOCK_PHASE
	uint8_t SPI_SoftwareSlaveManagement;		// see @SPI_SOFTWARE_SLAVE_MANAGEMENT
} SPI_Config_t;

/********************************
 * Handle structure for GPIO pins
 ********************************/
typedef struct {
	SPI_RegDef_t *pSPIx;				// holds the register struct for a SPI peripheral
	SPI_Config_t SPI_Config;			// holds user configurations for an SPI peripheral

} SPI_Handle_t;

/****************************
 * SPI Peripheral Definitions
 ****************************/

/* @SPI_DEVICE_MODES
 * SPI Device Modes (master or slave)
 */
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE			0

/* @SPI_BUS_CONFIGURATION
 * SPI Bus Configuration
 */
#define SPI_BUS_CONFIG_DUPLEX					0
#define SPI_BUS_CONFIG_HALF_DUPLEX				1
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY			2

/* @SPI_SERIAL_CLOCK_SPEED
 * SPI Serial Clock Speed based on baud rate control
 */
#define SPI_SCLK_FPCLK_2 			0
#define SPI_SCLK_FPCLK_4 			1
#define SPI_SCLK_FPCLK_8 			2
#define SPI_SCLK_FPCLK_16 			3
#define SPI_SCLK_FPCLK_32 			4
#define SPI_SCLK_FPCLK_64 			5
#define SPI_SCLK_FPCLK_128 			6
#define SPI_SCLK_FPCLK_256 			7

/* @SPI_DATA_FRAME_FORMAT
 * SPI Data Frame Format
 */
#define SPI_DATA_FRAME_8_BIT 	0
#define SPI_DATA_FRAME_16_BIT 	1

/* @SPI_CLOCK_POLARITY
 * SPI CPOL
 */
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

/* @SPI_CLOCK_PHASE
 * SPI CPHA
 */
#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1

/* @SPI_SOFTWARE_SLAVE_MANAGEMENT
 * SPI Software Slave Management
 */
#define SPI_SOFTWARE_SLAVE_DISABLE			0
#define SPI_SOFTWARE_SLAVE_ENABLE			1

/************************
 * Driver API
 ************************/
// Clock Control
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enable_flag);

// Peripheral Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Config_t *pSPIx);

// Transmit and Receive
void SPI_SendData(SPI_Config_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_Config_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

// IRQ Config and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); // note that the interrupt for that GPIO will be triggered and it knows what pin was activated


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
