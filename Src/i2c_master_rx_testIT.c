/*
 * i2c_master_rx_test.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

// I2C Defines
#define I2C_MASTER_ADDR 	0x61
#define I2C_SLAVE_ADDR 		0x68

// Globals
I2C_Handle_t I2C1Handle;
uint8_t RxCompleteFlag = 0;
/*
 * PUPD LED control
 */
void delay(int value) {
	for (uint32_t i=0; i<value; i++);
}

/*
 * PB8 = SCL
 * PB9 = SDA
 * Alt Func mode = 4
 */
void I2C1_GPIO_Init(void) {
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;

	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OPEN_DRAIN;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULL_UP;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&I2CPins);
}

void I2C1_Init(void) {
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = I2C_MASTER_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_CYCLE_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_Button_Init(void) {
	GPIO_Handle_t GPIO_Button_Handle;
	memset(&GPIO_Button_Handle, 0 , sizeof(GPIO_Handle_t));
	GPIO_Button_Handle.pGPIOx = GPIOC;

	GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;

	GPIO_Init(&GPIO_Button_Handle);
}

int main(void) {
	/* This will be an interrupt based version of master rx test
	 * 	Here we initialize interrupts. Note that priority config is not needed as we are only working with one interrupt
	 * 	We also need the interrupt name from the startup code. We override the IRQ handlers with our IRQ handler function
	 */
	initialise_monitor_handles();
	printf("Semihosting successful\n");
	GPIO_Button_Init();
	I2C1_GPIO_Init();
	I2C1_Init();

	// IRQ
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);
	// enable ack ater PE = 1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	uint8_t data_length = 0;
	uint8_t commandcode = 0;
	uint8_t RxBuffer[32];
	while (1) {
		// wait till button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		// get data length
		commandcode = 0x51;
		while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, I2C_SLAVE_ADDR, I2C_REPEATED_START_ENABLE) != I2C_STATE_READY);

		// receive data length from the arduino
		while (I2C_MasterReceiveDataIT(&I2C1Handle, &data_length, 1, I2C_SLAVE_ADDR, I2C_REPEATED_START_ENABLE) != I2C_STATE_READY);
		printf("The size of the data is %d\n", data_length);

		commandcode = 0x52;
		while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, I2C_SLAVE_ADDR, I2C_REPEATED_START_ENABLE) != I2C_STATE_READY);

		// receive the entirety of the data from the arduino
		while (I2C_MasterReceiveDataIT(&I2C1Handle, RxBuffer, data_length, I2C_SLAVE_ADDR, I2C_REPEATED_START_DISABLE) != I2C_STATE_READY);
		RxCompleteFlag = RESET;
		while(RxCompleteFlag == SET) {}

		RxBuffer[data_length+1] = '\0';
		printf("The message is %s\n", RxBuffer);
		RxCompleteFlag = RESET;
	}

	return 0;
}
void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}

// handle the events caused the by driver as described by the application events macros
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {
	if (AppEv == I2C_EV_TX_CMPLT) {
		printf("Tx is completed\n");
	} else if (AppEv == I2C_EV_RX_CMPLT) {
		printf("Rx is completed\n");
		RxCompleteFlag = SET;
	} else if (AppEv == I2C_EV_STOP) {
		printf("STOP bit cleared\n");
		// Set by hardware when a Stop condition is detected on the bus by the slave after an
		// acknowledge (if ACK=1)
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	} else if (AppEv == I2C_ERROR_AF) {
		printf("Error: ACK failure\n");
		// this failure happens when the slave fails to send an ack for the byte sent from the master
		// this failure stalls the transmission so we should close and reset the transmission process
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		while(1);
	} else if (AppEv == I2C_ERROR_BERR) {
		printf("Error: Bus Error\n");
		// Set by hardware when the interface detects an SDA rising or falling edge while SCL is high,
		// occurring in a non-valid position during a byte transfer.
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	} else if (AppEv == I2C_ERROR_ARLO) {
		printf("Error: ARLO Error\n");
		// Set by hardware when the interface loses the arbitration of the bus to another master
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	} else if (AppEv == I2C_ERROR_OVR) {
		printf("Error: OVR Error\n");
		// set by hardware in slave mode when an overrun or underrun error occurs
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	} else if (AppEv == I2C_ERROR_TIMEOUT) {
		printf("Error: TIMEOUT Error\n");
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}
