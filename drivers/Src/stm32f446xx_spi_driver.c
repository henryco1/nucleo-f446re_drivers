/*
 * stm32f446xx_spi_driver.c
 *
 *      Author: henryco1
 */
#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle();
static void spi_rxe_interrupt_handle();
static void spi_ovr_err_interrupt_handle();

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
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

/*
 * SPI Receive Data
 * desc: reads data from the SPI transmit buffer
 * input1: SPI register struct mapped to the SPI base address
 * input2: a buffer for holding information that goes into the rxbuffer
 * input3: the size of the transmission in bytes
 * output: none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while (len > 0) {
		// 1. wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_STATUS_RXNE_FLAG) == FLAG_RESET);

		// 2. check the DFF register
		// 3. then load the data into the data register
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bits
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len =- 2;
			(uint16_t*)pRxBuffer++;
		} else {
			// 8 bits
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

/************************************
 * Data Transmission Helper Functions
 ************************************/

/*
 * SPI Flag Status Helper
 * desc: checks the status of a user specified register
 * input1: SPI register struct ideally mapped to the status register
 * input2: an uint32_t status to check for
 * output: the flag register's status
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
 * 		when software slave management is enabled
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

/*
 * SPI SSOE Control
 * desc: controls the SSOE pin which has two configurations for the NSS pin
 *		SPI peripherals in master mode. 0 for single master and 1 for multi-master
 *		configuration. Note that when SPE = 1, SSOE = 0 when SSOE is enabled
 * input1: SPI register struct mapped to the SPI base address
 * input2: an ENABLE/DISABLE macro
 * output: none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*********************
 * Interrupt Functions
 *********************/

/*
 * SPI IRQ Interrupt Config
 * desc: configures NVIC registers for SPI peripherals
 * input1: 2 byte IRQ number
 * input2: a macro to enable/disable NVIC register
 * output: none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*
 * SPI IRQ Interrupt Priority Configuration
 * desc: configures the priority of an interrupt
 * input1: a uint8_t of the interrupt number
 * input2: a uint32_t for the IRQ priority number
 * output: none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t IPR_offset = IRQNumber % 4;
	uint8_t IPR_number = IRQNumber / 4;

	// stm32 nucleo f446re has 16 programmable priority levels (only 4 bits are used)
	uint8_t shift_amount = ( 8 * IPR_offset) + ( 8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPR_number) |= ( IRQPriority << shift_amount );
}

/*
 * SPI IRQ Handler
 * desc: handles interrupt events for errors, RX, TX events
 * note that the interrupt for that GPIO will be triggered and it knows what pin was activated
 * input1: an SPI handle containing information to handle IRQ events
 * output: none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	uint8_t temp1, temp2;

	// txe handler
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2) {
		spi_txe_interrupt_handle();
	}

	// rxe handler
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2) {
		spi_rxe_interrupt_handle();
	}

	// err handler
	// ONLY HANDLING OVR
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		spi_ovr_err_interrupt_handle();
	}
}

/*
 * SPI Send Data Non-Blocking
 * desc: sends data to the SPI transmit buffer via a non-blocking call using interrupts
 * input1: SPI register struct mapped to the SPI base address
 * input2: a buffer for holding information that goes into the txbuffer
 * input3: the size of the transmission in bytes
 * output: SPI peripheral state
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_STATE_BUSY_IN_TX) {
		// 1. save buffer information
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		// 2. reserve the SPI peripheral for ourselves by marking it as busy
		pSPIHandle->TxState = SPI_STATE_BUSY_IN_TX;
		// 3. enable TXEIE (SPI interrupt control bit) in SPIx_CR2
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/*
 * SPI Receive Data Non-Blocking
 * desc: reads data from the SPI transmit buffer via a non-blocking call using interrupts
 * input1: SPI register struct mapped to the SPI base address
 * input2: a buffer for holding information that goes into the rxbuffer
 * input3: the size of the transmission in bytes
 * output: SPI peripheral state
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_STATE_BUSY_IN_RX) {
		// 1. save buffer information
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		// 2. reserve the SPI peripheral for ourselves by marking it as busy
		pSPIHandle->RxState = SPI_STATE_BUSY_IN_RX;
		// 3. enable RXEIE (SPI interrupt control bit) in SPIx_CR2
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/****************************
 * Interrupt Helper functions
 ****************************/

/*
 * SPI overrun bit flag clearing helper
 * desc: clears the overrun error flag in the status register without a compiler warning
 * input1: SPI peripheral register definition struct
 * output: none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	/*
	 * Resetting ovr error bit involves reading from dr and sr registers
	 */
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*
 * SPI transmission closing helper function
 * desc: ends an SPI transmission
 * input1: SPI_Handle_t containing data to handle data transmit
 * output: none
 */
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_STATE_READY;
}

/*
 * SPI reception closing helper function
 * desc: ends SPI reception
 * input1: SPI_Handle_t containing data to handle data transmit
 * output: none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= (1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_STATE_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}

/******************
 * Static Functions
 ******************/

/*
 * SPI spi txe interrupt handler
 * desc: sends data to the SPI transmit buffer via a non-blocking call using interrupts
 * input1: SPI_Handle_t containing data to handle data transmit
 * output: none
 */
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	/*
	 * 1. wait until TXE is set
	 * 2. check the DFF register
	 * 3. then load the data into the data register
	 *
	 * Note no loop since the interrupt is meant to be called again and again
	 */
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bits
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen =- 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		// 8 bits
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if (!pSPIHandle->TxLen) {
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*
 * SPI spi rxe interrupt handler
 * desc: reads data from the SPI receive buffer via a non-blocking call using interrupts
 * input1: SPI_Handle_t containing data to handle data transmit
 * output: none
 */
void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	/*
	 * 1. wait until RXNE is set
	 * 2. check the DFF register
	 * 3. then load the data into the data register
	 *
	 * Note no loop since the interrupt is meant to be called again and again
	 */
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bits
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen =- 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	} else {
		// 8 bits
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if (!pSPIHandle->RxLen) {
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*
 * SPI spi ovr err interrupt handler
 * desc: resets the ovr err bit and notifies the caller
 * input1: SPI_Handle_t containing data to handle data transmit
 * output: none
 */
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
