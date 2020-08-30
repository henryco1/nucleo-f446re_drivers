/*
 * spi_tx_test.c
 *
 *      Author: henryco1
 */

#include "stm32f446xx.h"
#include <string.h>

/*
 * PB14 = SPI2_MISO
 * PB15 = SPI2_MOSI
 * PB13 = SPI2_SCLK
 * PB12 = SPI2_NSS
 * ALT Function Mode = 5
 */

void SPI_GPIOInit(void) {
	GPIO_Handle_t SPI_GPIOB;
	SPI_GPIOB.pGPIOx = GPIOB;

	SPI_GPIOB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_GPIOB.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPI_GPIOB.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	SPI_GPIOB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;
	SPI_GPIOB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

//	// MISO
//	SPI_GPIOB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&SPI_GPIOB);

	// MOSI
	SPI_GPIOB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI_GPIOB);

	// SCLK
	SPI_GPIOB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI_GPIOB);

//	// NSS
//	SPI_GPIOB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
//	GPIO_Init(&SPI_GPIOB);
}

void SPI2_Init(void) {
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_DUPLEX;
	SPI2Handle.SPI_Config.SPI_DataFrameFormat = SPI_DATA_FRAME_8_BIT;
	SPI2Handle.SPI_Config.SPI_SerialClkSpeed = SPI_SCLK_FPCLK_2;	//8 Mhz clock
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SoftwareSlaveManagement = SPI_SOFTWARE_SLAVE_ENABLE;

	SPI_Init(&SPI2Handle);
}

int main(void) {
	char data[] = "Hello world";

	// initialization
	SPI_GPIOInit();
	SPI2_Init();

	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

	while(SPI_GetFlagStatus(SPI2, SPI_STATUS_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

