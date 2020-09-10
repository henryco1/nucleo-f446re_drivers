/*
 * stm32f446xx_i2c_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_i2c_driver.h"

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
	uint8_t temp_reg = 0;

	/*
	 * Enable the ACK signals
	 */
	temp_reg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = temp_reg;

	/*
	 * Configure I2C with the peripheral clocks frequency
	 * 	only 5 bits are valid
	 */
	temp_reg = 0;
	temp_reg |= RCC_getPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = temp_reg & 0x3F;

	/*
	 * Specify the address of the device we are going to communicate with
	 */
	temp_reg = 0;
	temp_reg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp_reg |= (1 << I2C_OAR1_BIT14);
	pI2CHandle->pI2Cx = temp_reg;

	/*
	 * Configure the speed of the serial clock
	 */


	/*
	 * Configure the rise time for I2C pins
	 */

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

	uint32_t clk_src, clk_vco, plln = 0;
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

	temp = (RCC->PLLCFGR << 6) & 0x1FF;
	plln = temp;

	temp = (RCC->PLLCFGR << 0) & 0x3F;
	pllm = temp;

	clk_vco = clk_src * (plln / pllm);

	temp = (RCC->PLLCFGR << 16) & 0x3;
	pllp = pll_div_factors[temp];

	return clk_vco / pllp;
}

uint32_t RCC_getPCLK1Value(void) {
	uint32_t clk_sys, clk_ahb = 0;
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

	return (clk_sys / clk_ahb) / clk_apb;
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
