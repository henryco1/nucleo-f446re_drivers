/*
 * stm32f446xx_USART_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_USART_driver.h"

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
	} else {
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

/*****************************
 * Data Transmission Functions
 *****************************/
/*
 * USART Master Send Data
 * desc: a function for a master device to send data through USART to a slave device
 * input1: a pointer to an USART handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be sent
 * input4: uint8 slave address data
 * input5: an enable/disable flag for repeated starts
 * output: none
 */
void USART_TransmitData(USART_Handle_t pUSARTHandle, uint8_t *pTxBuffer, uint32_t len) {
	// enable usart (enable UE)
	pUSARTHandle->pUSARTx->CR1 &= (1 << USART_CR1_UE);
	// enable transmitter (TE)
	pUSARTHandle->pUSARTx->CR1 &= (1 << USART_CR1_TE);

	// write data
	while (len--) {
		// wait for the data transmission to finish
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)) {}

		// handle 9-bit and 8-bit frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_9_BITS) {
			uint16_t *largeData = (uint16_t*) pxTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*largeData & (uint16_t)0x01FF);

			// also need to handle parity bit config
			if (pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_CTRL_DISABLE) {
				// no parity used, therefore all 9 bits contain message data
				*pTxBuffer += 2;
			}
			else {
				// otherwise, the parity bit is enabled and the HARDWARE will add the 9th bit. So only send 8 bits
				*pTxBuffer++
			}
		}
		else {
			// if its just 8 bits, load a bit and increment the buffer ptr
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			*pTxBuffer++;
		}
	}

	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)) {}
}

/*
 * USART Master Receive Data
 * desc: a function for a master device to receive data through USART from a slave device
 * input1: a pointer to an USART handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be sent
 * input4: uint8 slave address data
 * input5: an enable/disable flag for repeated starts
 * output: none
 */
void USART_ReceiveData(USART_Handle_t pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
	// enable receiver (RE)
}

uint8_t USART_TransmitDataIT(USART_Handle_t pUSARTHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint8_t out;
	return out;
}
uint8_t USART_ReceiveDataIT(USART_Handle_t pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t out;
	return out;
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
void USART_IRQHandling(uint8_t pinNumber) {
	// clear the exti pr register corresponding to the pin number
	if (EXTI->PR & ( 1 << pinNumber )) {
		EXTI->PR |= ( 1 << pinNumber);
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
