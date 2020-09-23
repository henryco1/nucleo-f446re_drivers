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
	initialise_monitor_handles();
	printf("Semihosting successful\n");
	GPIO_Button_Init();
	I2C1_GPIO_Init();
	I2C1_Init();
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
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, I2C_SLAVE_ADDR, I2C_REPEATED_START_ENABLE);
		delay(500000);

		// receive data length from the arduino
		I2C_MasterReceiveData(&I2C1Handle, &data_length, 1, I2C_SLAVE_ADDR, I2C_REPEATED_START_ENABLE);
		printf("The size of the data is %d\n", data_length);
		delay(500000);

		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, I2C_SLAVE_ADDR, I2C_REPEATED_START_ENABLE);
		delay(500000);

		// receive the entirety of the data from the arduino
		I2C_MasterReceiveData(&I2C1Handle, RxBuffer, data_length, I2C_SLAVE_ADDR, I2C_REPEATED_START_DISABLE);
		RxBuffer[data_length+1] = '\0';
		printf("The message is %s\n", RxBuffer);
		delay(500000);
	}

	return 0;
}
