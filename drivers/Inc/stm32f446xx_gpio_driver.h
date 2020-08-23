/*
 * stm32f446xx_gpio_driver.h
 *
 *      Author: henryco1
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/***************************************
 * Configuration Structure for GPIO pins
 ***************************************/
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;						// see values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;						// see values from @GPIO_PIN_SPEED_MODES
	uint8_t GPIO_PinPuPdControl;				// see values from @GPIO_PUPDR_MODES
	uint8_t GPIO_PinOPType;						// see values from @GPIO_PORT_OUTPUT_MODES
	uint8_t GPIO_PinAltFuncMode;
} GPIO_PinConfig_t;

/********************************
 * Handle structure for GPIO pins
 ********************************/
typedef struct {
	GPIO_RegDef_t *pGPIOx; 						// base address pointer
	GPIO_PinConfig_t GPIO_PinConfig; 			// holds GPIO pin configuration settings

} GPIO_Handle_t;

/**********************
 * GPIO Specific Macros
 **********************/

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

/* @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_INPUT 			0
#define GPIO_MODE_OUTPUT 			1
#define GPIO_MODE_ALTFN 			2
#define GPIO_MODE_ANALOG 			3
// Made up modes
#define GPIO_MODE_INPUT_FALLING_EDGE						4
#define GPIO_MODE_INPUT_RISING_EDGE							5
#define GPIO_MODE_INPUT_RISING_EDGE_FALLING_EDGE			6

/* @GPIO_PIN_SPEED_MODES
 * GPIO output port speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/* @GPIO_PUPDR_MODES
 * GPIO port pull-up/pull down modes
 */
#define GPIO_PUPDR_NONE			0
#define GPIO_PUPDR_PULL_UP		1
#define GPIO_PUPDR_PULL_DOWN	2

/* @GPIO_PORT_OUTPUT_MODES
 * GPIO port output type
 */
#define GPIO_OTYPE_PUSH_PULL	0
#define GPIO_OTYPE_OPEN_DRAIN	1

/* @SYSCFG_GPIO_EXTI_CONFIGS
 * GPIO to EXTI config mappings
 */
#define GPIO_BASEADDR_TO_EXTI_CONFIG(GPIOx) ((GPIOx == GPIOA) ? 0 :\
											(GPIOx == GPIOB) ? 1 :\
											(GPIOx == GPIOC) ? 2 :\
											(GPIOx == GPIOD) ? 3 :\
											(GPIOx == GPIOE) ? 4 :\
											(GPIOx == GPIOF) ? 5 :\
											(GPIOx == GPIOG) ? 6 :\
											(GPIOx == GPIOH) ? 7 :0)

/* @NVIC_IRQ_PRIORITY_CONFIGS
 * IRQ number to NVIC config mappings
 */
#define IRQ_NUM_TO_NVIC_CONFIG(IRQx) 		((IRQx == 0) ? NVIC_IPR0 :\
											(IRQx == 1) ? NVIC_IPR1 :\
											(IRQx == 2) ? NVIC_IPR2 :\
											(IRQx == 3) ? NVIC_IPR3 :0)

/************************
 * Driver API
 ************************/

// Clock Control
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable_flag);

// Peripheral Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// GPIO Reading and Writing
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

// IRQ Config and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber); // note that the interrupt for that GPIO will be triggered and it knows what pin was activated

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
