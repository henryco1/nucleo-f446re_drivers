/*
 * i2c_slave_tx_string.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

//extern void initialise_monitor_handles(void);

// I2C Defines
#define I2C_SLAVE_ADDR 		0x69

// Globals
I2C_Handle_t I2C1Handle;
uint8_t TxBuffer[32] = "Hello World.....";

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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = I2C_SLAVE_ADDR;
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
//	initialise_monitor_handles();
//	printf("Semihosting successful\n");
	GPIO_Button_Init();
	I2C1_GPIO_Init();
	I2C1_Init();

	// IRQ
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
	I2C_SlaveConfigureCallBackEvents(I2C1, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);
	// enable ack ater PE = 1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

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
	static uint8_t commandCode;
	static uint8_t count = 0;

	if (AppEv == I2C_EV_DATA_REQ) {
		// master requests data from the slave
		if (commandCode == 0x51) {
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)TxBuffer));
//			printf("sending length info to master\n");
		} else if (commandCode == 0x52) {
			I2C_SlaveSendData(pI2CHandle->pI2Cx, TxBuffer[count++]);
//			printf("sending the contents of tx_buf\n");
		}

	} else if (AppEv == I2C_EV_DATA_RCV) {
		// master sends data to the slave
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
//		printf("The command code is: %x\n", commandCode);
	} else if (AppEv == I2C_ERROR_AF) {
		// nack sent by master, which is a signal to end data transfer. This event only happens during slave txing
		// invalidate the global vars command code and reset count.
		commandCode = 0xff;
		count = 0;
		printf("nack sent\n");
	} else if (AppEv == I2C_EV_STOP) {
		// only happens during slave reception. Master has ended I2C communication with the slave
//		printf("Master has ended the I2C communication with the slave.\n");

	}
}
