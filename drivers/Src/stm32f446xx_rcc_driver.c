/*
 * stm32f446xx_rcc_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_rcc_driver.h"

/*************************
 * Clock Control functions
 *************************/
/*
 * RCC Get PLL Output Clock
 * desc: gets the pll output clock
 * input1: none
 * output: a pll clock value to be used as a PCLK
 */
uint32_t RCC_GetPLLOutputClock(void) {
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

	temp = (RCC->PLLCFGR << RCC_PLLCFGR_PLLSRC) & 1;
	if (temp) {
		// HSI clock selected as PLL
		clk_src = 16000000;
	} else {
		// HSE clock selected as PLL
		clk_src = 8000000;
	}

	plln = (RCC->PLLCFGR << RCC_PLLCFGR_PLLN) & 0x1FF;
	pllm = (RCC->PLLCFGR << RCC_PLLCFGR_PLLM) & 0x3F;

	clk_vco = clk_src * (plln / pllm);

	temp = (RCC->PLLCFGR << RCC_PLLCFGR_PLLP) & 0x3;
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

	temp = (RCC->CFGR << RCC_CFGR_SWS) & 0x3;

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

	temp = (RCC->CFGR << RCC_CFGR_HPRE) & 0xF;
	if (temp < 8) clk_ahb = 1;
	else clk_ahb = hpre_div_factors[temp - 8];

	temp = (RCC->CFGR << RCC_CFGR_PPRE1) & 0x7;
	if (temp < 4) clk_apb = 1;
	else clk_apb = ppre1_div_factors[temp - 4];

	clk_pclk1 = (clk_sys / clk_ahb) / clk_apb;
	return clk_pclk1;
}

/*
 * RCC Get PLL PCLK2 Value
 * desc: gets the pclk value from the HSI oscillator. Contains unused capabilities to get PCLK from HSE or PLL
 * input1: none
 * output: a pll clock value to be used as a PCLK
 */
uint32_t RCC_GetPCLK2Value(void) {
	uint32_t clk_sys, clk_ahb = 0, clk_pclk2;
	uint8_t clk_apb = 0;
	uint8_t temp = 0;
	uint32_t hpre_div_factors[8] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint8_t ppre2_div_factors[4] = {2, 4, 8, 16};

	temp = (RCC->CFGR << RCC_CFGR_SWS) & 0x3;

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

	temp = (RCC->CFGR << RCC_CFGR_HPRE) & 0xF;
	if (temp < 8) clk_ahb = 1;
	else clk_ahb = hpre_div_factors[temp - 8];

	temp = (RCC->CFGR << RCC_CFGR_PPRE2) & 0x7;
	if (temp < 4) clk_apb = 1;
	else clk_apb = ppre2_div_factors[temp - 4];

	clk_pclk2 = (clk_sys / clk_ahb) / clk_apb;
	return clk_pclk2;
}
