/*
 * stm32f446xx_i2c_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
	// message is 8 bits which includes a slave addr and an instruction (r/w)
	slave_addr = slave_addr << 1;
	slave_addr &= ~(1);
	pI2Cx->DR = slave_addr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
	// message is 8 bits which includes a slave addr and an instruction (r/w)
	slave_addr = slave_addr << 1;
	slave_addr |= 1;
	pI2Cx->DR = slave_addr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	// SR->ADDR is cleared by reading SR1 and SR2
	uint32_t temp = pI2Cx->SR1;
	temp = pI2Cx->SR2;
	(void) temp;
}

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

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enable_flag) {
	if(enable_flag == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}

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

// Peripheral Init
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp_reg = 0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

//	/*
//	 * Enable the ACK signals
//	 */
//	temp_reg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
//	pI2CHandle->pI2Cx->CR1 = temp_reg;

	/*
	 * Configure I2C with the peripheral clocks frequency
	 * 	only 5 bits are valid
	 */
	temp_reg = 0;
	temp_reg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = temp_reg & 0x3F;

	/*
	 * Specify the address of the device we are going to communicate with
	 */
	temp_reg = 0;
	temp_reg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp_reg |= (1 << I2C_OAR1_BIT14);
	pI2CHandle->pI2Cx->OAR1 = temp_reg;

	/*
	 * Configure the speed of the serial clock
	 */
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

	/*
	 * Configure the rise time for I2C pins through TRISE
	 */
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		temp_reg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else {
		temp_reg = ((RCC_GetPCLK1Value() * 300) / 1000000000U + 1);
	}

	pI2CHandle->pI2Cx->TRISE = (temp_reg & 0x3F);


}

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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slave_addr) {
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm the start condition. Note that SB is cleared by reading. SCL is low until SB is cleared
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_SB_FLAG) );

	// send the address and the instruction
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slave_addr);

	// validate that the address was sent by checking ADDR bit
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_ADDR_FLAG) );

	// SCL will be held low until ADDR cleared
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// send data
	while(len--) {
		while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_TXE_FLAG) );
		// note that DR and pTxBuffer are both uint32_t
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
	}

	// when the transmission is complete, wait for TXE=1 and BTF=1 before sending the STOP
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_TXE_FLAG) );
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_BTF_FLAG) );

	// send the stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slave_addr) {
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
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// read from buffer
		for ( uint32_t i = len ; i > 0 ; i--) {
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_STATUS_RXNE_FLAG) );
			if(i == 2) {
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
//		while (len--) {
//			// wait for RXNE
//			while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_STATUS_RXNE_FLAG) );
//			if (len > 1) {
//				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
//				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
//			}
//			*pRxBuffer = pI2CHandle->pI2Cx->DR;
//			pRxBuffer++;
//		}
	}
	// reenable ACK
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag_name) {
	if (pI2Cx->SR1 & flag_name) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t enable_flag) {
	if(enable_flag == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}
