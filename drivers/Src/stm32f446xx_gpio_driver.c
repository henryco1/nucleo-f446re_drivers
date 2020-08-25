/*
 * stm32f446xx_gpio_driver.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx_gpio_driver.h"

/*
 * GPIO Clock Control
 * desc: enables or disables the clock for a given GPIO peripheral
 * input1: GPIO register struct mapped to the GPIO base address
 * input2: an ENABLE/DISABLE macro
 * output: none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable_flag) {
	if (enable_flag == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN;
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN;
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN;
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN;
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN;
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN;
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN;
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN;
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI;
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI;
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI;
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI;
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI;
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI;
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI;
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI;
		}
	}
}

/*
 * GPIO Initialization Function
 * desc: initializes a GPIOx peripheral
 * input1: GPIO handle struct containing config and address information
 * output: none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t curr_reg = 0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//configure gpio pin mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		curr_reg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= curr_reg;
		curr_reg = 0;
	}
	else {
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_FALLING_EDGE) {
			// 1. configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_RISING_EDGE) {
			// 1. configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_RISING_EDGE_FALLING_EDGE) {
			// 1. configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t reg_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t pin_offset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG->EXTICR[reg_index] &= ~(GPIO_BASEADDR_TO_EXTI_CONFIG(pGPIOHandle->pGPIOx) << (4 * pin_offset));

		// 3. enable the exti interrupt delivery using IMR (interrupt mask register)
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	// configure gpio speed
	curr_reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= curr_reg;
	curr_reg = 0;

	// configure pupd settings
	curr_reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= curr_reg;
	curr_reg = 0;

	// configure op type
	curr_reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= curr_reg;
	curr_reg = 0;

	// configure alt func
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode == GPIO_MODE_ALTFN) {
		// configure the alt function registers
		// since the alt register is handled as an array, we need to know the index and the pin within the index.
		uint32_t alt_index, pin_offset;
		alt_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		pin_offset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[alt_index] &= ~(0xF << (4 * pin_offset));
		pGPIOHandle->pGPIOx->AFR[alt_index] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * pin_offset));
	}
}

/*
 * GPIO Deinitialization Function
 * desc: cleans up and deinitializes a GPIOx peripheral
 * input1: GPIO register struct
 * output: none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_PCLK_RESET;
	}
	else if (pGPIOx == GPIOB) {
		GPIOB_PCLK_RESET;
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_PCLK_RESET;
	}
	else if (pGPIOx == GPIOD) {
		GPIOD_PCLK_RESET;
	}
	else if (pGPIOx == GPIOE) {
		GPIOE_PCLK_RESET;
	}
	else if (pGPIOx == GPIOF) {
		GPIOF_PCLK_RESET;
	}
	else if (pGPIOx == GPIOG) {
		GPIOG_PCLK_RESET;
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_PCLK_RESET;
	}
}

/*
 * GPIO Read From Input Pin
 * desc: reads the input pin of a GPIOx peripheral (specified by pinNumber)
 * input1: GPIO register struct mapped to the GPIO base address
 * input2: pin number
 * output: 2 byte pin register data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	uint8_t out;
	out = (uint8_t)((pGPIOx->IDR >> pinNumber) & 1);
	return out;
}

/*
 * GPIO Read From Input Port
 * desc: reads from an input port of a GPIOx peripheral
 * input1: GPIO register struct mapped to the GPIO base address
 * output: 4 byte GPIO port data
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t out;
	out = (uint16_t)((pGPIOx->IDR));
	return out;
}

/*
 * GPIO Write To Output Pin
 * desc: writes to a GPIOx output pin
 * input1: GPIO register struct mapped to the GPIO base address
 * input2: pin number
 * input3: a value to write into an output pin
 * output: none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
	// if we have something to write we write it. Otherwise the value must be zero, so we just
	// clear the reg
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << pinNumber);
	} else {
		pGPIOx->ODR &= ~(0xFF << pinNumber);
	}
}

/*
 * GPIO Write To Output Port
 * desc: writes to an output port of a GPIOx peripheral
 * input1: GPIO register struct mapped to the GPIO base address
 * output: none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

/*
 * GPIO Toggle Output Pin
 * desc: this function toggles a GPIOx output pin
 * input1: GPIO register struct mapped to the GPIO base address
 * input2: pin number
 * output: none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= (1 << pinNumber);
}

/*
 * GPIO Interrupt Request Config
 * desc: configures GPIOx IRQ
 * input1: 2 byte IRQ number
 * input2: 2 byte number representing the priority of the IRQ
 * input3: a macro to enable/disable GPIO
 * output: none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag) {
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
 * GPIO IRQ Interrupt Priority Configuration
 * desc: configures the priority of an interrupt
 * input1:
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t IPR_offset = IRQNumber % 4;
	uint8_t IPR_number = IRQNumber / 4;

	// stm32 nucleo f446re has 16 programmable priority levels (only 4 bits are used)
	uint8_t shift_amount = ( 8 * IPR_offset) + ( 8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPR_number) |= ( IRQPriority << shift_amount );
}

/*
 * GPIO Interrupt Request Handler
 * desc: handles GPIOx IRQ
 * input1: pin number
 * output: none
 */
void GPIO_IRQHandling(uint8_t pinNumber) {
	// clear the exti pr register corresponding to the pin number
	if (EXTI->PR & ( 1 << pinNumber )) {
		EXTI->PR |= ( 1 << pinNumber);
	}
}
