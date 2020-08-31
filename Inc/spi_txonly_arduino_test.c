/*
 * spi_txonly_arduino_test.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx.h"
#include <string.h>

/*
 * PUPD LED control
 */
void delay(int value) {
	for (uint32_t i=0; i<value; i++);
}

/*
 * PB12 = SPI2_NSS
 * PB13	= SPI2_SCLK
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

int main(void) {
	char data[] = "Hello Worldd a s asd q t s cg sg  sjg biy  7 9 d9 uhf9  adj ia if ia aop eiuy oa68 89 a 9";
	uint8_t data_size = strlen(data);

	// initialization
	GPIO_Button_Init();
	SPI2_GPIO_Init();
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {
		// wait till button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {}
		delay(500000);

		SPI_PeripheralControl(SPI2, ENABLE);

		// slave needs to know how much data is being sent. The slave script expect 1 byte of msg length then msg data
		SPI_SendData(SPI2, &data_size, 1);
		SPI_SendData(SPI2, (uint8_t*)data, data_size);

		while(SPI_GetFlagStatus(SPI2, SPI_STATUS_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}
