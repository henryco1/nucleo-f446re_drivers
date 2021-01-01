/*
 * stm32f446xx_USART_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_USART_driver.h"

static void USART_HandleTXEInterrupt(USART_Handle_t *pI2CHandle);
static void USART_HandleRXNEInterrupt(USART_Handle_t *pI2CHandle);

/*
 * USART Peripheral Clock Control
 * desc: enables the USART peripheral clock for a specific USART peripheral
 * input1: a pointer to an USART peripheral addr
 * input2: an enable or disable flag
 * output: none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN;
		}
		else if (pUSARTx == USART2) {
			USART2_PCLK_EN;
		}
		else if (pUSARTx == USART3) {
			USART3_PCLK_EN;
		}
		else if (pUSARTx == UART4) {
			UART4_PCLK_EN;
		}
		else if (pUSARTx == UART5) {
			UART5_PCLK_EN;
		}
		else if (pUSARTx == USART6) {
			USART6_PCLK_EN;
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI;
		}
		else if (pUSARTx == USART2) {
			USART2_PCLK_DI;
		}
		else if (pUSARTx == USART3) {
			USART3_PCLK_DI;
		}
		else if (pUSARTx == UART4) {
			UART4_PCLK_DI;
		}
		else if (pUSARTx == UART5) {
			UART5_PCLK_DI;
		}
		else if (pUSARTx == USART6) {
			USART6_PCLK_DI;
		}
	}
}

/*
 * USART Initialization Function
 * desc: initializes a USARTx peripheral
 * input1: USART handle struct containing config and address information
 * output: none
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/****************
	 *  Configure CR1
	 ****************/
	uint32_t temp = 0;

	// device mode
	if (pUSARTHandle->USART_Config.USART_DeviceMode == USART_DEVICE_MODE_TX_ONLY) {
		temp |= (1 << USART_CR1_TE);
	} else if (pUSARTHandle->USART_Config.USART_DeviceMode == USART_DEVICE_MODE_RX_ONLY) {
		temp |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USART_Config.USART_DeviceMode == USART_DEVICE_MODE_TX_RX) {
		temp |= (1 << USART_CR1_TE);
		temp |= (1 << USART_CR1_RE);
	}

	// word length
	temp |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	// parity
	if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_EVEN) {
		temp |= (1 << USART_CR1_PCE);
		// the even parity config (USART_CR1_PS == 0) is the default, no need to config
	} else if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_ODD) {
		temp |= (1 << USART_CR1_PCE);
		temp |= (1 << USART_CR1_PS);
	}

	// write config to CR1
	pUSARTHandle->pUSARTx->CR1 = temp;

	/****************
	 *  Configure CR2
	 ****************/
	temp = 0;

	// num of stop bits
	temp |= pUSARTHandle->USART_Config.USART_NumStopBits << USART_CR2_STOP;

	// write config to CR2
	pUSARTHandle->pUSARTx->CR2 = temp;

	/****************
	 *  Configure CR3
	 ****************/
	temp = 0;

	// hardware flow control mode
	if (pUSARTHandle->USART_Config.USART_HwFlowCtrl == USART_HW_FLOW_CTRL_CTS) {
		temp |= (1 << USART_CR3_CTSE);
	} else if (pUSARTHandle->USART_Config.USART_HwFlowCtrl == USART_HW_FLOW_CTRL_RTS) {
		temp |= (1 << USART_CR3_RTSE);
	} else if (pUSARTHandle->USART_Config.USART_HwFlowCtrl == USART_HW_FLOW_CTRL_CTS_RTS) {
		temp |= (1 << USART_CR3_CTSE);
		temp |= (1 << USART_CR3_RTSE);
	}

	// write config to CR3
	pUSARTHandle->pUSARTx->CR3 = temp;

	// TODO: init baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BaudRate);
}

/*
 * USART Deinitialization Function
 * desc: cleans up and deinitializes a USARTx peripheral
 * input1: USART register struct
 * output: none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx) {
	if (pUSARTx == USART1) {
		USART1_PCLK_DI;
	}
	else if (pUSARTx == USART2) {
		USART2_PCLK_DI;
	}
	else if (pUSARTx == USART3) {
		USART3_PCLK_DI;
	}
	else if (pUSARTx == UART4) {
		UART4_PCLK_DI;
	}
	else if (pUSARTx == UART5) {
		UART5_PCLK_DI;
	}
	else if (pUSARTx == USART6) {
		USART6_PCLK_DI;
	}
}

/*
 * USART Set Baud Rate
 * desc: configures the baud rate through the BRR mantissa and fraction for standard USART
 * input1: USART register struct
 * input2: desired baud rate value
 * output: none
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate) {
	uint32_t PCLKx;
	uint32_t usartdiv;
	uint32_t mantissa;
	uint32_t fraction;
	uint32_t temp = 0;

	if (pUSARTx == USART1 || pUSARTx == USART6) {
		PCLKx = RCC_GetPCLK2Value();
	} else {
		PCLKx = RCC_GetPCLK1Value();
	}

	if (pUSARTx->CR1 & ~(1 << USART_CR1_OVER8)) {
		// 16 bit oversampling rate case (value = 0)
		usartdiv = ((PCLKx * 100) / (baudRate * (2 - 0) * 8));
	} else {
		// 8 bit oversampling rate case
		usartdiv = ((PCLKx * 100) / (baudRate * (2 - 1) * 8));
	}

	mantissa = usartdiv / 100;
	temp |= mantissa << 4;

	fraction = usartdiv - (mantissa * 100);

	if(pUSARTx->CR1 & ~( 1 << USART_CR1_OVER8)) {
		//over sampling by 16
		fraction = ((( fraction * 16) + 50) / 100 ) & ((uint8_t)0x0F);
	} else {
		//OVER8 = 1 , over sampling by 8
		fraction = ((( fraction * 8) + 50) / 100 )& ((uint8_t)0x07);

	}
	temp |= fraction;

	pUSARTx->BRR = temp;
}

/*****************************
 * Data Transmission Functions
 *****************************/
/*
 * USART Transmit Data
 * desc: function for USART data transmission
 * input1: a pointer to an USART handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to send
 * output: none
 */
void USART_TransmitData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len) {
//	// enable usart (enable UE)
//	pUSARTHandle->pUSARTx->CR1 &= (1 << USART_CR1_UE);
//	// enable transmitter (TE)
//	pUSARTHandle->pUSARTx->CR1 &= (1 << USART_CR1_TE);

	// write data
	while (len--) {
		// wait for the data transmission to finish
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)) {}

		// handle 9-bit and 8-bit frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_9_BITS) {
			uint16_t *largeData = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*largeData & (uint16_t)0x01FF);

			// also need to handle parity bit config
			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				// no parity used, therefore all 9 bits contain message data
				pTxBuffer += 2;
			} else {
				// otherwise, the parity bit is enabled and the HARDWARE will add the 9th bit. So only send 8 bits
				pTxBuffer++;
			}
		} else {
			// if its just 8 bits, load a bit and increment the buffer ptr
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)) {}
}

/*
 * USART Receive Data
 * desc: function for USART data reception
 * input1: a pointer to an USART handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be received
 * output: none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
	// enable receiver (RE)
	while(len--) {
		// wait for the data transmission to finish
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE)) {}

		// handle 9-bit and 8-bit frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_9_BITS) {

			// if parity is disabled, all 9 bits need to be gathered
			// otherwise, the 9th bit will be a parity bit and only 8 bits will be relevant data
			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF);
				pRxBuffer += 2;
			} else {
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint16_t)0xFF);
				pRxBuffer++;
			}
		} else {
			// for 8 bit data frames, if parity is disabled, we want to read all 8 bits
			// otherwise, the 8th bit is a parity bit, and 7 bits will be relevant data
			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			} else {
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)) {}
}

/*
 * USART Send Data Non-Blocking
 * desc: sends data to the USART transmit buffer via a non-blocking call using interrupts
 * input1: USART register struct mapped to the USART base address
 * input2: a buffer for holding information that goes into the txbuffer
 * input3: the size of the transmission in bytes
 * output: USART peripheral state
 */
uint8_t USART_TransmitDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint8_t busy_state = pUSARTHandle->TxState;
	if((busy_state != USART_BUSY_IN_TX)) {
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = len;
		pUSARTHandle->TxState = USART_BUSY_IN_TX;

		// trigger the start condition and enable the interrupts, which generates an interrupt
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}
	return busy_state;
}

/*
 * USART Receive Data Non-Blocking
 * desc: reads data from the USART transmit buffer via a non-blocking call using interrupts
 * input1: USART register struct mapped to the USART base address
 * input2: a buffer for holding information that goes into the rxbuffer
 * input3: the size of the transmission in bytes
 * output: USART peripheral state
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t busy_state = pUSARTHandle->RxState;
	if((busy_state != USART_BUSY_IN_RX)) {
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = len;
		pUSARTHandle->RxState = USART_BUSY_IN_RX;

		// trigger the start condition and enable the interrupts, which generates an interrupt
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}
	return busy_state;
}
/*********************
 * Interrupt Functions
 *********************/
/*
 * USART IRQ Interrupt Config
 * desc: configures NVIC registers for USART peripherals
 * input1: 2 byte IRQ number
 * input2: a macro to enable/disable NVIC register
 * output: none
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag) {
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
 * USART IRQ Interrupt Priority Configuration
 * desc: configures the priority of an interrupt
 * input1: a uint8_t of the interrupt number
 * input2: a uint32_t for the IRQ priority number
 * output: none
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t IPR_offset = IRQNumber % 4;
	uint8_t IPR_number = IRQNumber / 4;

	// stm32 nucleo f446re has 16 programmable priority levels (only 4 bits are used)
	uint8_t shift_amount = ( 8 * IPR_offset) + ( 8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPR_number) |= ( IRQPriority << shift_amount );
}

/*
 * USART IRQ Handler
 * desc: handles interrupt events for errors, RX, TX events
 * note that the interrupt for that GPIO will be triggered and it knows what pin was activated
 * input1: an USART handle containing information to handle IRQ events
 * output: none
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {
	uint8_t temp1, temp2;

	// txe handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR1_TXEIE);
	if (temp1 && temp2) {
		USART_HandleTXEInterrupt(pUSARTHandle);
	}

	// rxne handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR1_RXNEIE);
	if (temp1 && temp2) {
		USART_HandleRXNEInterrupt(pUSARTHandle);
	}

	// ore handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR1_RXNEIE);
	if (temp1 && temp2) {
//		usart_ore_interrupt_handle();
	}

	// pe handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_PE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR1_PEIE);
	if (temp1 && temp2) {
//		usart_pe_interrupt_handle();
	}

	// cts handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR3_CTSIE);
	if (temp1 && temp2) {
//		usart_cts_interrupt_handle();
	}

	// tc handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR1_TCIE);
	if (temp1 && temp2) {
//		usart_tc_interrupt_handle();
	}

	// idle handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR1_IDLEIE);
	if (temp1 && temp2) {
//		usart_idle_interrupt_handle();
	}

	// lbdl handler
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR2_LBDL);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR2_LBDIE);
	if (temp1 && temp2) {
//		usart_lbdl_interrupt_handle();
	}

	// eie handler
	uint8_t NF = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_NF);
	uint8_t FE = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_FE);
	uint8_t PE = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_PE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_CR3_EIE);
	if ((NF || FE || PE) && temp2) {
//		usart_eie_interrupt_handle();
	}
}

/************************************
 * Data Transmission Helper Functions
 ************************************/
/*
 * USART Get Flag Status Helper
 * desc: checks the status of a user specified register
 * input1: USART register pointer mapped to an USART peripheral
 * input2: an uint32_t status to check for
 * output: the flag register's status
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag_name) {
    if (pUSARTx->SR & flag_name) {
    	return SET;
    }
   return RESET;
}

/*
 * USART Clear Flag Status Helper
 * desc: clears the status of a user specified register
 * input1: USART register pointer mapped to an USART peripheral
 * input2: an uint32_t flag to clear
 */
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag_name) {
	pUSARTx->SR &= ~(flag_name << FLAG_RESET);
}

/*
 * USART Clear Flag Status Helper
 * desc: clears the status of a user specified register
 * input1: USART register pointer mapped to an USART peripheral
 * input2: an uint8_t flag for enabling or disabling
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*
 * USART transmission closing helper function
 * desc: ends an USART transmission
 * input1: USART_Handle_t containing data to handle data transmit
 * output: none
 */
void USART_CloseTransmission(USART_Handle_t *pUSARTHandle) {
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
	pUSARTHandle->pTxBuffer = NULL;
	pUSARTHandle->TxLen = 0;
	pUSARTHandle->TxState = USART_EVENT_IDLE;
}

/*
 * USART reception closing helper function
 * desc: ends USART reception
 * input1: USART_Handle_t containing data to handle data transmit
 * output: none
 */
void USART_CloseReception(USART_Handle_t *pUSARTHandle) {
	pUSARTHandle->pUSARTx->CR1 &= (1 << USART_CR1_RXNEIE);
	pUSARTHandle->pRxBuffer = NULL;
	pUSARTHandle->RxLen = 0;
	pUSARTHandle->RxState = USART_EVENT_IDLE;
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}

/******************
 * Static Functions
 ******************/
/*
 * USART Handle TXE Interrupt
 * desc: interrupt function that handles USART data transmission
 * input1: pointer to an USART handle struct
 * output: none
 */
static void USART_HandleTXEInterrupt(USART_Handle_t *pUSARTHandle) {
	if (pUSARTHandle->TxLen > 0) {
		// 8 bit word length
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_8_BITS) {
			pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
			pUSARTHandle->TxLen--;
			pUSARTHandle->pTxBuffer++;
		}
		else {
			// 9 bit word length
			pUSARTHandle->pUSARTx->DR = *((uint16_t*)pUSARTHandle->pTxBuffer) & (uint16_t)0x1FF;

			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				// parity disabled
				pUSARTHandle->TxLen--;
				pUSARTHandle->pTxBuffer++;
			} else {
				// parity enabled
				pUSARTHandle->TxLen -= 2;
				(uint16_t*)pUSARTHandle->pTxBuffer++;
			}

		}
		if (!pUSARTHandle->TxLen) {
			USART_CloseTransmission(pUSARTHandle);
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
		}
	}
}

/*
 * USART Handle RNXE Interrupt
 * desc: interrupt function that handles USART data reception
 * input1: pointer to an USART handle struct
 * output: none
 */
static void USART_HandleRXNEInterrupt(USART_Handle_t *pUSARTHandle) {
	if (pUSARTHandle->RxLen > 0) {
		// 8 bit word length
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_8_BITS) {
			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			} else {
				*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pUSARTHandle->RxLen--;
			pUSARTHandle->pRxBuffer++;
		}
		else {
			// 9 bit word length
			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				// parity disabled
				*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF);
				pUSARTHandle->RxLen -= 2;
				(uint16_t*)pUSARTHandle->pRxBuffer++;
				(uint16_t*)pUSARTHandle->pRxBuffer++;
			} else {
			// parity enabled
				*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint16_t)0xFF);
				pUSARTHandle->RxLen--;
				pUSARTHandle->pRxBuffer++;
			}

		}
		if (!pUSARTHandle->RxLen) {
			USART_CloseTransmission(pUSARTHandle);
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
		}
	}
}
