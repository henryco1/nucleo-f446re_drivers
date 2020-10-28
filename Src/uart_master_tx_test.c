/*
 * uart_master_tx_test.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

//extern void initialise_monitor_handles(void);

// I2C Defines
//#define I2C_SLAVE_ADDR 		0x69

// Globals
USART_Handle_t USART2Handle;
uint8_t commandCode;
uint32_t data_len = 0;
//uint8_t TxBuffer[] = "Hello World.....I am a very nice robotic machine. I can help with reading books, brewing coffee, and carrying heavy items. I hope that we can become good friends...unless 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 uwu 0.0 :flushed:";
char msg[1024] = "UART Hello World\n";
/*
 * PUPD LED control
 */
void delay(int value) {
	for (uint32_t i=0; i<value; i++);
}

/*
 * PA2 = TX
 * PA3 = RX
 * Alt Func mode = 7
 */
void USART2_GPIO_Init(void) {
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;

	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL; // why push pull
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULL_UP;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&USARTPins);

	// RX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&USARTPins);
}

void USART2_Init(void) {
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_BaudRate = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_DeviceMode = USART_DEVICE_MODE_TX_ONLY;
	USART2Handle.USART_Config.USART_HwFlowCtrl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_NumStopBits = USART_NUM_STOP_BITS_1;
	USART2Handle.USART_Config.USART_ParityCtrl = USART_PARITY_CTRL_DISABLE;
	USART2Handle.USART_Config.USART_WordLength = USART_WORD_LENGTH_8_BITS;

	USART_Init(&USART2Handle);
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
	data_len = strlen((char*)msg);
	GPIO_Button_Init();
	USART2_GPIO_Init();
	USART2_Init();
	USART_PeripheralControl(USART2, ENABLE);

	while (1) {
		// wait for button press
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		// avoid button de-bounce
		delay(200000);

		USART_TransmitData(&USART2Handle, (uint8_t*)msg, data_len);
	}
	return 0;
}

//// handle the events caused the by driver as described by the application events macros
//void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {
//	static uint32_t count = 0;
//	static uint32_t w_ptr = 0;
//
//	if (AppEv == I2C_ERROR_AF) {
//		// nack sent by master, which is a signal to end data transfer. This event only happens during slave txing
//		// if the current active command code is 0x52, don't invalidate, there is more data to send
//		if (!(commandCode = 0x52))
//			commandCode = 0xff;
//		// reset the count var because its the end of the transmission
//		count = 0;
//		// slave concludes it sent all the bytes when w_ptr reaches data_len
//		if (w_ptr >= (data_len)) {
//			w_ptr = 0;
//			commandCode = 0xff;
//		}
//	} else if (AppEv == I2C_EV_STOP) {
//		// only happens during slave reception. Master has ended I2C communication with the slave
//		// slave concludes end of data transfer
//		count = 0;
//	} else if (AppEv == I2C_EV_DATA_REQ) {
//		// master requests data from the slave
//		if (commandCode == 0x51) {
//			// send 4 bytes of length data
//			I2C_SlaveSendData(I2C1, ((data_len >> (count % 4) * 8)) & 0xff);
//			count++;
//		} else if (commandCode == 0x52) {
//			// send tx buffer contents
//			I2C_SlaveSendData(I2C1, TxBuffer[w_ptr++]);
//		}
//	} else if (AppEv == I2C_EV_DATA_RCV) {
//			// master sends data to the slave
//			commandCode = I2C_SlaveReceiveData(I2C1);
//	}
//}
//
//void I2C1_EV_IRQHandler(void) {
//	I2C_EV_IRQHandling(&USART2Handle);
//}
//
//void I2C1_ER_IRQHandler(void) {
//	I2C_ER_IRQHandling(&USART2Handle);
//}
