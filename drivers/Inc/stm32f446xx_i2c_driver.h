/*
 * stm32f446xx_i2c_driver.h
 *
 *      Author: henryco1
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/***************************************
 * Configuration Structure for I2C pins
 ***************************************/
typedef struct {
	uint32_t I2C_SCLSpeed;				// see @I2C_SCL_SPEED
	uint8_t I2C_DeviceAddress;			// see @I2C_DEVICE_ADDRRESS
	uint8_t I2C_ACKControl;				// see @I2C_ACK_CONTROL
	uint8_t I2C_FMDutyCycle;			// see @I2C_FM_DUTY_CYCLE
} I2C_Config_t;

/********************************
 * Handle structure for I2C pins
 ********************************/
typedef struct {
	I2C_RegDef_t *pI2Cx;				// holds the register struct for a I2C peripheral
	I2C_Config_t I2C_Config;			// holds user configurations for an I2C peripheral
} I2C_Handle_t;

/****************************
 * I2C Peripheral Definitions
 ****************************/

/* @I2C_SCL_SPEED
 * I2C SCL Clock Speed Configurations
 */
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM_2K			200000
#define I2C_SCL_SPEED_FM_4K			400000

/* @I2C_ACK_CONTROL
 * I2C ACK message enable
 */
#define I2C_ACK_DISABLE				0
#define I2C_ACK_ENABLE				1

/* @I2C_FM_DUTY_CYCLE
 * I2C Fast Mode Clock Duty Cycle
 */
#define I2C_FM_DUTY_CYCLE_2				0
#define I2C_FM_DUTY_CYCLE_16_9			1

/************************
 * Driver API
 ************************/
// Clock Control
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t enable_flag);

// Peripheral Init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// Transmit and Receive


// IRQ Config and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle); // note that the interrupt for that GPIO will be triggered and it knows what pin was activated

/*
 * Other control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enable_flag);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag_name);

/*
 * Application callback
 */

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
