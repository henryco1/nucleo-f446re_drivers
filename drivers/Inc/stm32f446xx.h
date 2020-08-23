/*
 * stm32f446xx.h
 *
 *      Author: henryco1
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#define __vo volatile

/**************************************
 * Processor Registers
 **************************************/
/*
 * NVIC interrupt set enable register
 */
#define NVIC_ISER0				((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*) 0xE000E10C)

/*
 * NVIC interrupt clear enable register
 */
#define NVIC_ICER0				((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*) 0xE000E18C)

/*
 * NVIC interrupt priority register
 */
#define NVIC_IPR0				((__vo uint32_t*) 0xE000E400)
#define NVIC_IPR1				((__vo uint32_t*) 0xE000E404)
#define NVIC_IPR2				((__vo uint32_t*) 0xE000E408)
#define NVIC_IPR3				((__vo uint32_t*) 0xE000E40C)

/*
 * ARM Cortex M4 NVIC Priority Register Address
 */
#define NVIC_PR_BASE_ADDR		((__vo uint32_t*) 0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NUM_PR_BITS_IMPLEMENTED  4

// Memory Macros
#define FLASH_BASEADDR 			0x08000000U	/* see memory mapping in datasheet or embedded flash memory in ref manual */
#define SRAM1_BASEADDR			0x20000000U	// 112kb
#define SRAM2_BASEADDR			0x2001C000U	// 16kb
#define ROM						0x1FFF0000U	/* referred to as system memory */
#define SRAM 					SRAM1_BASEADDRU

// Bus Domains
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

// AHB1 Peripherals
// Note that base addresses are preferred here, save the offsets for specific peripherals
#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x00000000U)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x00000400U)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00000800U)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00000C00U)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00001000U)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00001400U)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00001800U)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00001C00U)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x00003800U)

// APB1 Peripherals
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x00005400U)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x00005800U)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x00005C00U)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x00003800U)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x00003C00U)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x00004400U)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x00004800U)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x00004C00U)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x00005000U)

// APB2 Peripherals
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x00003000U)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x00003400U)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x00001000U)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x00001400U)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x00003C00U)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x00003800U)

/*******************************************
 * Peripheral Register Definition Structures
 *******************************************/

/*
 * Register map for GPIO
 */
typedef struct {
	__vo uint32_t MODER;			// GPIO port mode 											offset: 0x00
	__vo uint32_t OTYPER;			// GPIO port output type									offset: 0x04
	__vo uint32_t OSPEEDER;			// GPIO port output speed									offset: 0x08
	__vo uint32_t PUPDR;			// GPIO port pull-up/pull down 								offset: 0x0C
	__vo uint32_t IDR;				// GPIO port input data										offset: 0x10
	__vo uint32_t ODR;				// GPIO port output data									offset: 0x14
	__vo uint32_t BSRR;				// GPIO port bit set/reset									offset: 0x18
	__vo uint32_t LCKR;				// GPIO port config lock									offset: 0x1C
	__vo uint32_t AFR[2];			// GPIO port alt func low and high							offset: 0x20
} GPIO_RegDef_t;

/*
 * Register map for SPI
 */
typedef struct {
	__vo uint32_t CR1;				// SPI control 1											offset: 0x00
	__vo uint32_t CR2;				// SPI control 2 											offset: 0x04
	__vo uint32_t SR;				// SPI status												offset: 0x08
	__vo uint32_t DR;				// SPI data		 											offset: 0x0C
	__vo uint32_t CRCPR;			// SPI CRC polynomial										offset: 0x10
	__vo uint32_t RXCRCR;			// SPI receive CRC polynomal								offset: 0x14
	__vo uint32_t TXCRCR;			// SPI transmit CRC polynomal								offset: 0x18
	__vo uint32_t I2SCFGR;			// SPI I2S config											offset: 0x1C
	__vo uint32_t I2SPR;			// SPI I2S prescaler										offset: 0x20
} SPI_RegDef_t;

/*
 * Register map for RCC
 */
typedef struct {
	__vo uint32_t CR;				// RCC clock control 										offset: 0x00
	__vo uint32_t PLLCFGR;			// RCC PLL config											offset: 0x04
	__vo uint32_t CFGR;				// RCC clock config											offset: 0x08
	__vo uint32_t CIR;				// RCC clock interrupt										offset: 0x0C
	__vo uint32_t AHB1RSTR;			// RCC AHB1 peripheral reset 								offset: 0x10
	__vo uint32_t AHB2RSTR;			// RCC AHB2 peripheral reset								offset: 0x14
	__vo uint32_t AHB3RSTR;			// RCC AHB3 peripheral reset								offset: 0x18
	uint32_t RESERVED0;				// reserved													offset: 0x1C
	__vo uint32_t APB1RSTR;			// RCC APB1 peripheral reset								offset: 0x20
	__vo uint32_t APB2RSTR;			// RCC APB2 peripheral reset								offset: 0x24
	uint32_t RESERVED1[2];			// reserved													offset: 0x28
									// reserved													offset: 0x2C
	__vo uint32_t AHB1ENR;			// RCC AHB1 clock enable 									offset: 0x30
	__vo uint32_t AHB2ENR;			// RCC AHB2 clock enable									offset: 0x34
	__vo uint32_t AHB3ENR;			// RCC AHB3 clock enable									offset: 0x38
	uint32_t RESERVED2;				// reserved													offset: 0x3C
	__vo uint32_t APB1ENR;			// RCC APB1 clock enable									offset: 0x40
	__vo uint32_t APB2ENR;			// RCC APB2 clock enable									offset: 0x44
	uint32_t RESERVED3[2];			// reserved													offset: 0x48
									// reserved													offset: 0x4C
	__vo uint32_t AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode		offset: 0x50
	__vo uint32_t AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode		offset: 0x54
	__vo uint32_t AHB3LPENR;		// RCC AHB3 peripheral clock enable in low power mode		offset: 0x58
	uint32_t RESERVED4;				// reserved													offset: 0x5C
	__vo uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode		offset: 0x60
	__vo uint32_t APB2LPENR;		// RCC APB2 peripheral clock enable in low power mode		offset: 0x64
	uint32_t RESERVED5[2];			// reserved													offset: 0x68
									// reserved													offset: 0x6C
	__vo uint32_t BDCR;				// RCC backup domain control								offset: 0x70
	__vo uint32_t CSR;				// RCC clock control and status								offset: 0x74
	uint32_t RESERVED6[2];			// reserved													offset: 0x78
									// reserved													offset: 0x7C
	__vo uint32_t SSCGR;			// RCC spread spectrum clock generation register			offset: 0x80
	__vo uint32_t PLLI2SCFGR;		// RCC PLLI2S config										offset: 0x84
	__vo uint32_t PLLSAICFGR;		// RCC PLL config											offset: 0x88
	__vo uint32_t DCKCFGR;			// RCC dedicated clock config 								offset: 0x8C
	__vo uint32_t CKGATENR;			// RCC gated enable											offset: 0x90
	__vo uint32_t DCKCFGR2;			// RCC dedicated clock config 2								offset: 0x94
} RCC_RegDef_t;

/*
 * Register map for EXTI
 */
typedef struct {
	__vo uint32_t IMR;				// EXTI interrupt mask										offset: 0x00
	__vo uint32_t EMR;				// EXTI event mask											offset: 0x04
	__vo uint32_t RTSR;				// EXTI rising trigger selection							offset: 0x08
	__vo uint32_t FTSR;				// EXTI falling trigger selection							offset: 0x0C
	__vo uint32_t SWIER;			// EXTI software interrupt event							offset: 0x10
	__vo uint32_t PR;				// EXTI pending register									offset: 0x14
} EXTI_RegDef_t;

/*
 * Register map for SYSCFG
 */
typedef struct {
	__vo uint32_t MEMRMP;				// SYSCFG memory remap 									offset: 0x00
	__vo uint32_t PMC;					// SYSCFG peripheral mode config 						offset: 0x04
	__vo uint32_t EXTICR[4];			// SYSCFG external interrupt config1 					offset: 0x08
										// SYSCFG external interrupt config2 					offset: 0x0C
										// SYSCFG external interrupt config3 					offset: 0x10
										// SYSCFG external interrupt config4 					offset: 0x14
	uint32_t RESERVED1[2];				// reserved												offset: 0x18
									    // reserved												offset: 0x1C
	__vo uint32_t CMPCR;				// SYSCFG compensation cell control 					offset: 0x20
	uint32_t RESERVED2[2];				// reserved												offset: 0x24
										// reserved												offset: 0x28
	__vo uint32_t CFGR;					// SYSCFG config 										offset: 0x2C

} SYSCFG_RegDef_t;

/************************
 * Peripheral Definitions
 ************************/
#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * Clock Enable Macros
 */

// GPIOx Clock Enables
#define GPIOA_PCLK_EN			( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN			( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN			( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN			( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN			( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN			( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN			( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN			( RCC->AHB1ENR |= (1 << 7) )

// I2Cx Clock Enables
#define I2C1_PCLK_EN			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN			( RCC->APB1ENR |= (1 << 23) )

// SPIx Clock Enables
#define SPI1_PCLK_EN			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN			( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN			( RCC->APB2ENR |= (1 << 13) )

// USARTx Clock Enables
#define USART1_PCLK_EN			( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN			( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN			( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN			( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN			( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN			( RCC->APB2ENR |= (1 << 5) )

// SYSCFG Clock Enables
#define SYSCFGEN_PCLK_EN		( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Disable Macros
 */

// GPIOx Clock Disables
#define GPIOA_PCLK_DI			( RCC->AHB1ENR &= (0 << 0) )
#define GPIOB_PCLK_DI			( RCC->AHB1ENR &= (0 << 1) )
#define GPIOC_PCLK_DI			( RCC->AHB1ENR &= (0 << 2) )
#define GPIOD_PCLK_DI			( RCC->AHB1ENR &= (0 << 3) )
#define GPIOE_PCLK_DI			( RCC->AHB1ENR &= (0 << 4) )
#define GPIOF_PCLK_DI			( RCC->AHB1ENR &= (0 << 5) )
#define GPIOG_PCLK_DI			( RCC->AHB1ENR &= (0 << 6) )
#define GPIOH_PCLK_DI			( RCC->AHB1ENR &= (0 << 7) )

// I2Cx Clock Disables
#define I2C1_PCLK_DI			( RCC->APB1ENR &= (0 << 21) )
#define I2C2_PCLK_DI			( RCC->APB1ENR &= (0 << 22) )
#define I2C3_PCLK_DI			( RCC->APB1ENR &= (0 << 23) )

// SPIx Clock Disables
#define SPI1_PCLK_DI			( RCC->APB2ENR &= (0 << 12) )
#define SPI2_PCLK_DI			( RCC->APB1ENR &= (0 << 14) )
#define SPI3_PCLK_DI			( RCC->APB1ENR &= (0 << 15) )
#define SPI4_PCLK_DI			( RCC->APB2ENR &= (0 << 13) )

// USARTx Clock Disables
#define USART1_PCLK_DI			( RCC->APB2ENR &= (0 << 4) )
#define USART2_PCLK_DI			( RCC->APB1ENR &= (0 << 17) )
#define USART3_PCLK_DI			( RCC->APB1ENR &= (0 << 18) )
#define UART4_PCLK_DI			( RCC->APB1ENR &= (0 << 19) )
#define UART5_PCLK_DI			( RCC->APB1ENR &= (0 << 20) )
#define USART6_PCLK_DI			( RCC->APB2ENR &= (0 << 5) )

// SYSCFG Clock Disables
#define SYSCFGEN_PCLK_DI		( RCC->APB2ENR &= (0 << 14) )

/*
 * Clock Reset Macros
 */

#define GPIOA_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 0) ); ( RCC->AHB1RSTR &= ~(1 << 0) ); } while (0)
#define GPIOB_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 1) ); ( RCC->AHB1RSTR &= ~(1 << 1) ); } while (0)
#define GPIOC_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 2) ); ( RCC->AHB1RSTR &= ~(1 << 2) ); } while (0)
#define GPIOD_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) ); } while (0)
#define GPIOE_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 4) ); ( RCC->AHB1RSTR &= ~(1 << 4) ); } while (0)
#define GPIOF_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 5) ); ( RCC->AHB1RSTR &= ~(1 << 5) ); } while (0)
#define GPIOG_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 6) ); ( RCC->AHB1RSTR &= ~(1 << 6) ); } while (0)
#define GPIOH_PCLK_RESET		do { ( RCC->AHB1RSTR |= (1 << 7) ); ( RCC->AHB1RSTR &= ~(1 << 7) ); } while (0)

/*
 * IRQ Numbers for the stm32f446xx MCU family
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * IRQ Priority Levels
 */
#define IRQ_PRI0				0
#define IRQ_PRI1				1
#define IRQ_PRI2				2
#define IRQ_PRI3				3
#define IRQ_PRI4				4
#define IRQ_PRI5				5
#define IRQ_PRI6				6
#define IRQ_PRI7				7
#define IRQ_PRI8				8
#define IRQ_PRI9				9
#define IRQ_PRI10				10
#define IRQ_PRI11				11
#define IRQ_PRI12				12
#define IRQ_PRI13				13
#define IRQ_PRI14				14
#define IRQ_PRI15				15

/*
 * Generic Macros
 */
#define ENABLE 					1
#define DISABLE					0
#define SET						ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

#include "stm32f446xx_gpio_driver.h"

#endif /* INC_STM32F446XX_H_ */
