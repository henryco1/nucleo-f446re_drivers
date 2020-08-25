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

	uint32_t reg_data = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. configure device mode
	reg_data |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. init spi bus configuration
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_DUPLEX) {
		// clear bidirectional mode
		reg_data &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX) {
		// enable bidirectional mode
		// keep RXONLY clear when bidirectional mode is active
		reg_data |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY) {
		// RXONLY and BIDIMODE can't be set at the same time, so configure RXONLY
		// handle RXONLY
//		reg_data &= ~(1 << SPI_CR1_BIDIMODE);
		reg_data |= (1 << SPI_CR1_RXONLY);
	}

	// 3. set clock speed
	reg_data |= (pSPIHandle->SPI_Config.SPI_SerialClkSpeed << SPI_CR1_BR);

	// 4. set data frame format
	reg_data |= (pSPIHandle->SPI_Config.SPI_DataFrameFormat << SPI_CR1_DFF);

	// 5. set cpol
	reg_data |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// 6. set cpha
	reg_data |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	// 7. configure software slave management
	reg_data |= (pSPIHandle->SPI_Config.SPI_SoftwareSlaveManagement<< SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = reg_data;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx) {
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

/*
 * SPI Send Data
 * desc: sends data to the SPI transmit buffer via a blocking call
 * input1: SPI register struct mapped to the SPI base address
 * input2: a buffer for holding information that goes into the txbuffer
 * input3: the size of the transmission in bytes
 * output: none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while (len > 0) {
		// 1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_STATUS_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFF register
		// 3. then load the data into the data register
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len =- 2;
			(uint16_t*)pTxBuffer++;
		} else {
			// 8 bits
			pSPIx->DR = pTxBuffer;
			len--;
			pTxBuffer++;
		}


	}
}

/*
 * SPI Receive Data
 * desc: sends data to the SPI transmit buffer
 * input1: SPI register struct mapped to the SPI base address
 * input2: a buffer for holding information that goes into the txbuffer
 * input3: the size of the transmission in bytes
 * output: none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

}

/*
 * SPI Flag Status Helper
 * desc: checks the status of a user specified register
 * input1: SPI register struct ideally mapped to the status register
 * input2: an uint32_t status to check for
 * output: none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag_name) {
	if (pSPIx->SR & flag_name) return FLAG_SET;
	return FLAG_RESET;
}

/*
 * SPI Peripheral Control
 * desc: enables the SPI peripheral for data transmission
 * input1: SPI register struct mapped to the SPI base address
 * input2: an ENABLE/DISABLE macro
 * output: none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * SPI SSI Control
 * desc: controls the SSI pin which controls the NSS pin for SPI peripherals
 * input1: SPI register struct mapped to the SPI base address
 * input2: an ENABLE/DISABLE macro
 * output: none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
// IRQ Config and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); // note that the interrupt for that GPIO will be triggered and it knows what pin was activated

