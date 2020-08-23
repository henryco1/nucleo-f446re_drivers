/*
 * stm32f446xx_spi_driver.c
 *
 *      Author: henryco1
 */
#include "stm32f446xx_spi_driver.h"

/*
 * SPI Clock Control
 * desc: enables or disables the clock for a given SPI peripheral
 * input1: SPI register struct mapped to the SPI base address
 * input2: an ENABLE/DISABLE macro
 * output: none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN;
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN;
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN;
		}
		else if (pSPIx == SPI4) {
			SPI4_PCLK_EN;
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI;
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI;
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_DI;
		}
		else if (pSPIx == SPI4) {
			SPI4_PCLK_DI;
		}
	}
}

// Peripheral Init
void SPI_Init(SPI_Handle_t *pSPIHandle) {
//	typedef struct {
//		uint8_t SPI_DeviceMode;						// see @SPI_DEVICE_MODES
//		uint8_t SPI_BusConfig;						// see @SPI_BUS_CONFIGURATION
//		uint8_t SPI_SerialClkSpeed;					// see @SPI_SERIAL_CLOCK_SPEED
//		uint8_t SPI_DataFrameFormat;				// see @SPI_DATA_FRAME_FORMAT
//		uint8_t SPI_CPOL;							// see @SPI_CLOCK_POLARITY
//		uint8_t SPI_CPHA;							// see @SPI_CLOCK_PHASE
//		uint8_t SPI_SoftwareSlaveManagement;		// see @SPI_SOFTWARE_SLAVE_MANAGEMENT
//	} SPI_Config_t;
	uint32_t curr_reg = 0;

	// 1. configure device mode
	if (pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
		pSPIHandle->pSPIx->CR1 &= (0 << 2);
		pSPIHandle->pSPIx->CR1 |= (1 << 2);
	}
	else {
		pSPIHandle->pSPIx->CR1 &= (0 << 2);
	}

	// 2. init spi bus configuration
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_DUPLEX) {
		// enable bidirectional mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);

		// seems that BIDIOE will need to be toggled when reading and writing

		// keep RXONLY clear when bidirectional mode is active
		pSPIHandle->pSPIx->CR1 &= ~(1 << 10);

	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX) {
		// enable bidirectional mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
		pSPIHandle->pSPIx->CR1 |= (1 << 15);

		// seems that BIDIOE will need to be toggled when reading and writing

		// keep RXONLY clear when bidirectional mode is active
		pSPIHandle->pSPIx->CR1 &= ~(1 << 10);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY) {
		// RXONLY and BIDIMODE can't be set at the same time, so configure RXONLY
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
		pSPIHandle->pSPIx->CR1 &= ~(1 << 14);

		// handle RXONLY
		pSPIHandle->pSPIx->CR1 &= ~(1 << 10);
		pSPIHandle->pSPIx->CR1 |= (1 << 10);
	}

	// 3. set clock speed


	// 4. set data frame format

	// 5. set cpol

	// 6. set cpha

	// 7. configure software slave management
}
void SPI_DeInit(SPI_Config_t *pSPIx);

// Transmit and Receive
void SPI_SendData(SPI_Config_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_Config_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

// IRQ Config and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); // note that the interrupt for that GPIO will be triggered and it knows what pin was activated

