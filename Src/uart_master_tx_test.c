/*
 * i2c_slave_tx_string2.c
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
uint8_t commandCode;
uint32_t data_len = 0;
uint8_t TxBuffer[] = "Hello World.....I am a very nice robotic machine. I can help with reading books, brewing coffee, and carrying heavy items. I hope that we can become good friends...unless 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 :flushed:";

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
//	initialise_monitor_handles();
//	printf("Semihosting successful\n");
	data_len = strlen((char*)TxBuffer);
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

	return 0;
}

// handle the events caused the by driver as described by the application events macros
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {
	static uint32_t count = 0;
	static uint32_t w_ptr = 0;

	if (AppEv == I2C_ERROR_AF) {
		// nack sent by master, which is a signal to end data transfer. This event only happens during slave txing
		// if the current active command code is 0x52, don't invalidate, there is more data to send
		if (!(commandCode = 0x52))
			commandCode = 0xff;
		// reset the count var because its the end of the transmission
		count = 0;
		// slave concludes it sent all the bytes when w_ptr reaches data_len
		if (w_ptr >= (data_len)) {
			w_ptr = 0;
			commandCode = 0xff;
		}
	} else if (AppEv == I2C_EV_STOP) {
		// only happens during slave reception. Master has ended I2C communication with the slave
		// slave concludes end of data transfer
		count = 0;
	} else if (AppEv == I2C_EV_DATA_REQ) {
		// master requests data from the slave
		if (commandCode == 0x51) {
			// send 4 bytes of length data
			I2C_SlaveSendData(I2C1, ((data_len >> (count % 4) * 8)) & 0xff);
			count++;
		} else if (commandCode == 0x52) {
			// send tx buffer contents
			I2C_SlaveSendData(I2C1, TxBuffer[w_ptr++]);
		}
	} else if (AppEv == I2C_EV_DATA_RCV) {
			// master sends data to the slave
			commandCode = I2C_SlaveReceiveData(I2C1);
	}
}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}
