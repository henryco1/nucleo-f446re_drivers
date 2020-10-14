/*
 * stm32f446xx_usart_driver.h
 *
 *      Author: henryco1
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/***************************************
 * Configuration Structure for USARTpins
 ***************************************/
typedef struct {
	uint8_t USART_DeviceMode;					// see values from @USART_DEVICE_MODES
	uint8_t USART_BaudRate;						// see values from @USART_BAUD_RATES
	uint8_t USART_NumStopBits;					// see values from @USART_NUM_STOP_BITS_VALUE
	uint8_t USART_WordLength;					// see values from @USART_WORD_LENGTHS
	uint8_t USART_ParityCtrl;					// see values from @USART_PARITY_CONTROL_VALUES
	uint8_t USART_HwFlowCtrl;					// see values from @USART_HW_FLOW_CTRL_MODES
} USART_Config_t;

/********************************
 * Handle structure for USART pins
 ********************************/
typedef struct {
	USART_RegDef_t *pUSARTx; 					// base address pointer
	USART_Config_t USART_Config; 			// holds USART pin configuration settings

} USART_Handle_t;

/**********************
 * USART Specific Macros
 **********************/

/*
 * @USART_DEVICE_MODES
 * USART device modes
 */
#define USART_DEVICE_MODE_TX_ONLY				0
#define USART_DEVICE_MODE_RX_ONLY				1
#define USART_DEVICE_MODE_TX_RX					2

/*
 * @USART_BAUD_RATES
 * Baud Rate Options
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/* @USART_NUM_STOP_BITS_VALUE
 * USART stop bit modes
 * Note: The 0.5 Stop bit and 1.5 Stop bit are not available for UART4 & UART5
 */
#define USART_NUM_STOP_BITS_1					0
#define USART_NUM_STOP_BITS_0_5					1
#define USART_NUM_STOP_BITS_2					2
#define USART_NUM_STOP_BITS_1_5					3

/* @USART_WORD_LENGTHS
 * USART word lengths
 */
#define USART_WORD_LENGTH_8_BITS			0
#define USART_WORD_LENGTH_9_BITS			1

/* @USART_PARITY_CONTROL_VALUES
 * USART parity control values
 */
#define USART_PARITY_CTRL_EVEN				2
#define USART_PARITY_CTRL_ODD				1
#define USART_PARITY_CTRL_DISABLE			0

/* @USART_HW_FLOW_CTRL_MODES
 * possible USART hardware flow control modes
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART register status flag macros
 */
#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * USART Application states
 */
#define USART_READY 			0
#define USART_BUSY_IN_RX 		1
#define USART_BUSY_IN_TX 		2

/*
 * USART Application states
 */
#define USART_EVENT_TX_CMPLT   	0
#define	USART_EVENT_RX_CMPLT   	1
#define	USART_EVENT_IDLE      	2
#define	USART_EVENT_CTS       	3
#define	USART_EVENT_PE        	4
#define	USART_ERR_FE     		5
#define	USART_ERR_NE    	 	6
#define	USART_ERR_ORE    		7

/************************
 * Driver API
 ************************/

// Clock Control
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t enable_flag);

// Peripheral Init
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

// USART Reading and Writing
void USART_TransmitData(USART_Handle_t pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_TransmitDataIT(USART_Handle_t pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);


// IRQ Config and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enable_flag);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(uint8_t pinNumber); // note that the interrupt for that USART will be triggered and it knows what pin was activated

// Data Transmission Helpers
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag_name);
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag_name);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t enable_flag);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
