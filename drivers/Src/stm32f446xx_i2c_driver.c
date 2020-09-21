/*
 * stm32f446xx_i2c_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_i2c_driver.h"

static void I2C_MasterHandleRNXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

/*************************
 * Clock Control functions
 *************************/
/*
 * RCC Get PLL Output Clock
 * desc: gets the pll output clock
 * input1: none
 * output: a pll clock value to be used as a PCLK
 */
uint32_t RCC_GetPLLOutputClock() {
	/*
	 * Formulas:
	 *	f (VCO clock) = f (PLL clock input) × (PLLN / PLLM)
	 *	f (PLL general clock output) = f (VCO clock) / PLLP
	 *
	 * Definitions:
	 * 	PLLP = main PLL division factor for main system clock
	 * 	PLLN = main PLL multiplication factor for VCO
	 * 	PLLM = division factor for the main PLL input clock
	 */
	uint8_t pll_div_factors[4] = {2, 4, 6, 8};

	uint32_t clk_src, clk_vco, plln, clk_pll = 0;
	uint8_t pllp, pllm = 0;
	uint8_t temp = 0;

	temp = (RCC->PLLCFGR << 22) & 1;
	if (temp) {
		// HSI clock selected as PLL
		clk_src = 16000000;
	} else {
		// HSE clock selected as PLL
		clk_src = 8000000;
	}

	plln = (RCC->PLLCFGR << 6) & 0x1FF;
	pllm = (RCC->PLLCFGR << 0) & 0x3F;

	clk_vco = clk_src * (plln / pllm);

	temp = (RCC->PLLCFGR << 16) & 0x3;
	pllp = pll_div_factors[temp];
	clk_pll = clk_vco / pllp;

	return clk_pll;
}

/*
 * RCC Get PLL PCLK1 Value
 * desc: gets the pclk value from the HSI oscillator. Contains unused capabilities to get PCLK from HSE or PLL
 * input1: none
 * output: a pll clock value to be used as a PCLK
 */
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t clk_sys, clk_ahb = 0, clk_pclk1;
	uint8_t clk_apb = 0;
	uint8_t temp = 0;
	uint32_t hpre_div_factors[8] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint8_t ppre1_div_factors[4] = {2, 4, 8, 16};

	temp = (RCC->CFGR << 2) & 0x3;

	if (temp == 0) {
		// HSI oscillator
		clk_sys = 16000000;
	} else if (temp == 1) {
		// HSE oscillator
		clk_sys = 8000000;
	} else if (temp == 2) {
		// PLL system clock
		clk_sys = RCC_GetPLLOutputClock();
	}

	temp = (RCC->CFGR << 4) & 0xF;
	if (temp < 8) clk_ahb = 1;
	else clk_ahb = hpre_div_factors[temp - 8];

	temp = (RCC->CFGR << 10) & 0x7;
	if (temp < 4) clk_apb = 1;
	else clk_apb = ppre1_div_factors[temp - 4];

	clk_pclk1 = (clk_sys / clk_ahb) / clk_apb;
	return clk_pclk1;
}

/*
 * I2C Peripheral Clock Control
 * desc: enables the I2C peripheral clock for a specific I2C peripheral
 * input1: a pointer to an I2C peripheral addr
 * input2: an enable or disable flag
 * output: none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN;
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN;
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN;
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI;
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI;
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI;
		}
	}
}
/****************************
 * Initialization functions
 ****************************/
/*
 * I2C Peripheral Initialization
 * desc: initializes an I2C peripheral
 * input1: a pointer to an I2C handle struct
 * output: none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp_reg = 0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Configure I2C with the peripheral clocks frequency
	// only 5 bits are valid
	temp_reg = 0;
	temp_reg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = temp_reg & 0x3F;

	// Specify the address of the device we are going to communicate with
	temp_reg = 0;
	temp_reg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp_reg |= (1 << I2C_OAR1_BIT14);
	pI2CHandle->pI2Cx->OAR1 = temp_reg;

	// Configure the speed of the serial clock
	uint16_t ccr_val = 0;
	temp_reg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// slow (standard) mode
		ccr_val = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp_reg |= (ccr_val & 0xFFF);
	} else {
		// fast mode
		temp_reg |= (1 << I2C_CCR_FS);
		temp_reg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

	}
	pI2CHandle->pI2Cx->CCR = temp_reg;


	// Configure the rise time for I2C pins through TRISE
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		temp_reg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else {
		temp_reg = ((RCC_GetPCLK1Value() * 300) / 1000000000U + 1);
	}

	pI2CHandle->pI2Cx->TRISE = (temp_reg & 0x3F);
}

/*
 * I2C Peripheral Deinitialization
 * desc: deinitializes an I2C peripheral
 * input1: a pointer to an I2C peripheral
 * output: none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_PCLK_DI;
	}
	else if (pI2Cx == I2C2) {
		I2C2_PCLK_DI;
	}
	else if (pI2Cx == I2C3) {
		I2C3_PCLK_DI;
	}
}

/*
 * I2C Peripheral Control
 * desc: enables or disables the I2C peripheral (which enables all I2C devices?)
 * input1: a pointer to an I2C peripheral addr
 * input2: an enable or disable flag
 * output: none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enable_flag) {
	if(enable_flag == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}
/************************************
 * Data Transmission Helper Functions
 ************************************/
/*
 * I2C Master Send Data
 * desc: a function for a master device to send data through I2C to a slave device
 * input1: a pointer to an I2C handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be sent
 * input4: uint8 slave address data
 * input5: an enable/disable flag for repeated starts
 * output: none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr, uint8_t repeated_start_flag) {
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm the start condition. Note that SB is cleared by reading. SCL is low until SB is cleared
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_SB_FLAG));

	// send the address and the instruction
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slave_addr);

	// validate that the address was sent by checking ADDR bit
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_ADDR_FLAG));

	// SCL will be held low until ADDR cleared
	I2C_ClearADDRFlag(pI2CHandle);

	// send data
	while(len--) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_TXE_FLAG));
		// note that DR and pTxBuffer are both uint32_t
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
	}

	// when the transmission is complete, wait for TXE=1 and BTF=1 before sending the STOP
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_TXE_FLAG));
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_BTF_FLAG));

	// send the stop condition
	if (repeated_start_flag == I2C_REPEATED_START_DISABLE ) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/*
 * I2C Master Receive Data
 * desc: a function for a master device to receive data through I2C from a slave device
 * input1: a pointer to an I2C handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be sent
 * input4: uint8 slave address data
 * input5: an enable/disable flag for repeated starts
 * output: none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr, uint8_t repeated_start_flag) {
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm the start condition
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_SB_FLAG) );

	// send the address and the instruction to the slave addr
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slave_addr);

	// validate that the addr was sent
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_ADDR_FLAG) );

	// handle the single byte receive data case
	if (len == 1) {
		// NACK before clearing addr
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		I2C_ClearADDRFlag(pI2CHandle);

		// wait for RXNE
		while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_RXNE_FLAG) );

		// send stop
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// read from buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	// handle reading multiple bytes
	if (len > 1) {
		// clear addr
		I2C_ClearADDRFlag(pI2CHandle);

		// read from buffer
		for (uint32_t i = len ; i > 0 ; i--) {
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_STATUS_RXNE_FLAG) );
			if(i == 2) {
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
				if (repeated_start_flag == I2C_REPEATED_START_DISABLE) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	// reenable ACK
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/*********************
 * Interrupt Functions
 *********************/
/*
 * I2C IRQ Master Send Data
 * desc: a nonblocking handler for a master device to send data to a slave device. This function enables the appropriate interrupts and
 * 		generates the start condition.
 * input1: a pointer to an I2C handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be sent
 * input4: uint8 slave address data
 * input5: an enable/disable flag for repeated starts
 * output: none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr, uint8_t repeated_start_flag) {
	uint8_t busy_state = pI2CHandle->TxRxState;
	if((busy_state != I2C_STATE_BUSY_IN_TX) && (busy_state != I2C_STATE_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_IN_TX;
		pI2CHandle->DeviceAddr = slave_addr;
		pI2CHandle->Sr = repeated_start_flag;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// Enable interrupts
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busy_state;
}

/*
 * I2C IRQ Master Receive Data
 * desc: a nonblocking handler for a master device to receive data from a slave device. This function enables the appropriate interrupts and
 * 		generates the start condition.
 * input1: a pointer to an I2C handle struct
 * input2: a uint8 pointer to a buffer
 * input3: the size of the message to be sent
 * input4: uint8 slave address data
 * input5: an enable/disable flag for repeated starts
 * output: none
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr, uint8_t repeated_start_flag) {
	uint8_t busy_state = pI2CHandle->TxRxState;

	if((busy_state != I2C_STATE_BUSY_IN_TX) && (busy_state != I2C_STATE_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_IN_RX;
		pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DeviceAddr = slave_addr;
		pI2CHandle->Sr = repeated_start_flag;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// Enable interrupts
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busy_state;
}

/*
 * I2C IRQ Interrupt Config
 * desc: enables or disables a specific interrupt for a peripheral function
 * input1: a number describing which interrupt to enable
 * input2: an flag for disabling or enabling an NVIC reg
 * output: none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag) {
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
 * I2C IRQ Priority Config
 * desc: configures the priority of an interrupt
 * input1: a uint8_t of the interrupt number
 * input2: a uint32_t for the IRQ priority number
 * output: none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t IPR_offset = IRQNumber % 4;
	uint8_t IPR_number = IRQNumber / 4;

	// stm32 nucleo f446re has 16 programmable priority levels (only 4 bits are used)
	uint8_t shift_amount = ( 8 * IPR_offset) + ( 8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPR_number) |= ( IRQPriority << shift_amount );
}

/*
 * I2C EV IRQ Handling
 * desc: Interrupt handling for both master and slave modes of a device. Handles all I2C interrupt events
 * input1: a pointer to an I2C handle struct
 * output: none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint8_t ITEVTEN_val = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	uint8_t ITBUFEN_val = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	uint8_t temp = 0;

	// check SB
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	// handle the interrupt generated by SB event
	if (ITEVTEN_val && temp) {
		if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX)
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DeviceAddr);
		else if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX)
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DeviceAddr);
	}

	// check ADDR
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	// handle the interrupt generated by the ADDR event
	if (ITEVTEN_val && temp) {
		// this function will handle the case of receiving the last byte
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// check STOPF
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	// handle the interrupt generated by the STOPF event. This will only be generated by the slave
	if (ITEVTEN_val && temp) {
		// to clear stop we need to read from SR1 and write to CR1. SR1 was already read above so we will write to SR1 here
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		// notify application about STOP
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// check BTF
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	// handle the interrupt generated by the BTF event. Note that this event will only be triggered
	// from a transmission
	if (ITEVTEN_val && temp) {
		if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX) {
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
				if (pI2CHandle->TxLen == 0) {
					// Closing out an interrupt based data transmission requires 3 different steps.
					// 1. Generate the stop condition
					if (pI2CHandle->Sr == I2C_REPEATED_START_DISABLE)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					// 2. Reset the handle struct
					I2C_CloseSendData(pI2CHandle);
					// 3. Notify the application that the transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			} else if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX) {}
		}
	}

	// check TxE
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	// handle the TxE interrupt event
	if (ITEVTEN_val && ITBUFEN_val && temp) {
		// check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
		else {
			// slave
		}
	}

	// check RxNE
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	// handle the RxNE interrupt event
	if (ITEVTEN_val && ITBUFEN_val && temp) {
		// check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			I2C_MasterHandleRNXEInterrupt(pI2CHandle);
		}
		else {
			// slave
		}
	}
}

/*
 * I2C ER IRQ Handling
 * desc: Interrupt handling for both master and slave modes of a device. Handles all I2C error events
 * input1: a pointer to an I2C handle struct
 * output: none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	// for error handling clear the status bit and inform the application
	uint8_t ITERREN_val = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);
	uint8_t temp = 0;

	// check BERR
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	// handle the interrupt generated by SB event
	if (ITERREN_val && temp) {
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}
	// check ARLO
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
	// handle the interrupt generated by SB event
	if (ITERREN_val && temp) {
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}
	// check AF, ack failure
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
	// handle the interrupt generated by SB event
	if (ITERREN_val && temp) {
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}
	// check OVR, overrun/underrun
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
	// handle the interrupt generated by SB event
	if (ITERREN_val && temp) {
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}
	// check TIMEOUT
	temp |= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
	// handle the interrupt generated by SB event
	if (ITERREN_val && temp) {
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/************************************
 * Data Transmission Helper Functions
 ************************************/
/*
 * I2C Flag Status Helper
 * desc: checks the status of a user specified register
 * input1: I2C register pointer mapped to an I2C peripheral
 * input2: an uint32_t status to check for
 * output: the flag register's status
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag_name) {
	if (pI2Cx->SR1 & flag_name) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * I2C ACK Manager
 * desc: handles the enabling/disabling of I2C ACKing
 * input1: I2C register pointer mapped to an I2C peripheral
 * input2: a flag for enabling/disabling
 * output: none
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t enable_flag) {
	if(enable_flag == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/**********************************************
 * Interrupt Data Transmission Helper Functions
 **********************************************/
/*
 * I2C Close Send Data
 * desc: resets the IRQ enables and clears the I2C handle data for the next data transmission
 * input1: pointer to an I2C handle struct
 * output: none
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_STATE_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*
 * I2C Close Receive Data
 * desc: resets the IRQ enables and clears the I2C handle data for the next data reception
 * input1: pointer to an I2C handle struct
 * output: none
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_STATE_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

/******************
 * Static Functions
 ******************/
/*
 * I2C Master Handle TXE Interrupt
 * desc: interrupt function that handles I2C data transmission
 * input1: pointer to an I2C handle struct
 * output: none
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->TxLen > 0) {
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer++;
		pI2CHandle->TxLen--;
	}
}

/*
 * I2C Master Handle RNXE Interrupt
 * desc: interrupt function that handles I2C data reception
 * input1: pointer to an I2C handle struct
 * output: none
 */
static void I2C_MasterHandleRNXEInterrupt(I2C_Handle_t *pI2CHandle) {
	// two cases: data reception and the last byte data reception
	if (pI2CHandle->RxLen > 1) {
		if (pI2CHandle->RxLen == 2) I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	} else if (pI2CHandle->RxLen == 1) {
		// the clear address function will handle the addr clearing before this function is called
		// so ack disable is not required here
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	} else if (pI2CHandle->RxLen == 0) {
		// generate stop
		if (pI2CHandle->Sr == I2C_REPEATED_START_DISABLE)
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// close I2C rx
		I2C_CloseReceiveData(pI2CHandle);
		// notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/*
 * I2C Generate Start Condition
 * desc: helper function that generates a start condition
 * input1: pointer to the I2C peripheral addr
 * output: none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*
 * I2C Generate Stop Condition
 * desc: helper function that generates a stop condition
 * input1: pointer to the I2C peripheral addr
 * output: none
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/*
 * I2C Execute Address Phase Write
 * desc: helper function that transmits the slave addr followed by a write command
 * input1: pointer to the I2C peripheral addr
 * input2: the slave device addr
 * output: none
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
	// message is 8 bits which includes a slave addr and an instruction (r/w)
	slave_addr = slave_addr << 1;
	slave_addr &= ~(1);
	pI2Cx->DR = slave_addr;
}

/*
 * I2C Execute Address Phase Read
 * desc: helper function that transmits the slave addr followed by a read command
 * input1: pointer to the I2C peripheral addr
 * input2: the slave device addr
 * output: none
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
	// message is 8 bits which includes a slave addr and an instruction (r/w)
	slave_addr = slave_addr << 1;
	slave_addr |= 1;
	pI2Cx->DR = slave_addr;
}

/*
 * I2C Clear Address Flag
 * desc: helper function that clears the ADDR bit in the in the SR1 reg
 * input1: pointer to an I2C handle struct
 * output: none
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t clear_read = 0;
	// check if device is master or slave mode
	// master mode conditional
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		if (pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				// disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// then clear addr
				// SR->ADDR is cleared by reading SR1 and SR2
				clear_read = pI2CHandle->pI2Cx->SR1;
				clear_read = pI2CHandle->pI2Cx->SR2;
				(void)clear_read;
			}
		} else {
			// if the peripheral is busy trying to transmit, perform the normal ADDR clear
			clear_read = pI2CHandle->pI2Cx->SR1;
			clear_read = pI2CHandle->pI2Cx->SR2;
			(void)clear_read;
		}
	} else { // slave mode conditional
		clear_read = pI2CHandle->pI2Cx->SR1;
		clear_read = pI2CHandle->pI2Cx->SR2;
		(void)clear_read;
	}
}
