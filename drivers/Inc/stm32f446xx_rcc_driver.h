/*
 * stm32f446xx_rcc_driver.h
 *
 *      Author: henryco1
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/************************
 * Driver API
 ************************/
// Clock Control
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
