/*
 * spi_cmd_handling.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

// Slave Command Codes
#define CMD_LED_CTRL			0x50
#define CMD_SENSOR_READ			0x51
#define CMD_LED_READ			0x52
#define CMD_PRINT				0x53
#define CMD_ID_READ				0x54

#define LED_ON		1
#define LED_OFF		0

// Arduino Analog Pins
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

// Arduino LED
#define LED_PIN 		9

/*
 * PUPD LED control
 */
void delay(int value) {
	for (uint32_t i=0; i<value; i++);
}

/*
 * PB12 = SPI2_NSS
 * PB13	= SPI2_SCLK
 * PB14 = SPI2_MISO
 * PB15 = SPI2_MOSI
 * Alt Func mode = 5
 */
void SPI2_GPIO_Init(void) {
	GPIO_Handle_t GPIOB_Handle;
	memset(&GPIOB_Handle, 0 , sizeof(GPIO_Handle_t));
	GPIOB_Handle.pGPIOx = GPIOB;

	GPIOB_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULL_UP;
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// NSS
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GPIOB_Handle);

	// SCLK
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&GPIOB_Handle);

	// MISO
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GPIOB_Handle);

	// MOSI
	GPIOB_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIOB_Handle);
}

void SPI2_Init(void) {
	SPI_Handle_t SPI2_Handle;
	memset(&SPI2_Handle, 0 , sizeof(SPI_Handle_t));
	SPI2_Handle.pSPIx = SPI2;

	SPI2_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_DUPLEX;
	SPI2_Handle.SPI_Config.SPI_DataFrameFormat = SPI_DATA_FRAME_8_BIT;
	SPI2_Handle.SPI_Config.SPI_SoftwareSlaveManagement = SPI_SOFTWARE_SLAVE_DISABLE;
	SPI2_Handle.SPI_Config.SPI_SerialClkSpeed = SPI_SCLK_FPCLK_16;
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;

	SPI_Init(&SPI2_Handle);
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
int SPI_VerifyResponse(uint8_t response_byte){
	if (response_byte == 0xF5) {
		//ack
		return 1;
	}
	return 0;
}
int main(void) {
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	// initialization
	initialise_monitor_handles();
	printf("Semihosting successful\n");
	GPIO_Button_Init();
	SPI2_GPIO_Init();
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {
		// wait till button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL		<pin no(1)>		<value(1)>
		uint8_t command_code = CMD_LED_CTRL;
		uint8_t ack_byte;
		uint8_t args[2];

		SPI_SendData(SPI2, &command_code, 1);
		// dummy read to clear the RXbuffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy data to get a response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// then call receive
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if (SPI_VerifyResponse(ack_byte)) {
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send arguments
			SPI_SendData(SPI2, args, 2);
		}
		// end of CMD_LED_CTRL

		// 2. CMD_SENSOR_READ
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		command_code = CMD_SENSOR_READ;
		SPI_SendData(SPI2, &command_code, 1);
		// dummy read to clear the RXbuffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy data to get a response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// then call receive
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if (SPI_VerifyResponse(ack_byte)) {
			args[0] = ANALOG_PIN0;

			// send arguments
			SPI_SendData(SPI2, args, 1);
			// dummy read to clear the RXbuffer
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			delay(500000);
			// send dummy data to get a response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);
			// read from the analog pin
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);

		}
		// end of CMD_SENSOR_READ

		// 3. CMD_LED_READ
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		command_code = CMD_LED_READ;
		SPI_SendData(SPI2, &command_code, 1);
		// dummy read to clear the RXbuffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy data to get a response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// then call receive
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if (SPI_VerifyResponse(ack_byte)) {
			args[0] = LED_PIN;

			// send arguments
			SPI_SendData(SPI2, args, 1);
			// dummy read to clear the RXbuffer
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			delay(900000);
			// send dummy data to get a response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);
			// read from the led pin
			uint8_t led_read;
			SPI_ReceiveData(SPI2, &led_read, 1);
			printf("CMD_LED_READ %d\n",led_read);

		}
		// end of CMD_LED_READ

		// 4. CMD_PRINT
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		command_code = CMD_PRINT;
		SPI_SendData(SPI2, &command_code, 1);
		// dummy read to clear the RXbuffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy data to get a response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// then call receive
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		uint8_t message[] = "Hello World";
		if (SPI_VerifyResponse(ack_byte)) {
			// we only need to send data, no need for receive after handshake
			args[0] = strlen((char*)message);

			// send arguments
			SPI_SendData(SPI2, args, 1);
			// send message, no need to get data from the slave
			SPI_SendData(SPI2, message, args[0]);
			printf("CMD_PRINT complete\n");

		}
		// end of CMD_LED_READ

		// 5. CMD_ID_READ
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		command_code = CMD_ID_READ;
		SPI_SendData(SPI2, &command_code, 1);
		// dummy read to clear the RXbuffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy data to get a response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// then call receive
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		uint8_t pin_id[11];
		if (SPI_VerifyResponse(ack_byte)) {
			// note that arduinos have a 10 byte id

			for (uint32_t i=0; i<11; i++) {
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &pin_id[i], 1);
			}
			pin_id[11] = '\0';
			printf("CMD_ID_READ %s\n", pin_id);

		}
		// end of CMD_LED_READ

		while(SPI_GetFlagStatus(SPI2, SPI_STATUS_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}

