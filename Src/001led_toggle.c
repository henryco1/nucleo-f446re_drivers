/*
 * 001led_toggle.c
 *
 *      Author: henryco1
 */
#include "stm32f446xx.h"

/*
 * PUPD LED control
 */
void delay(void) {
	for (uint32_t i=0; i<500000; i++);
}

int main(void) {

	GPIO_Handle_t gpioLED;
	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	while(1) {
		GPIO_ToggleOutputPin(GPIOA, gpioLED.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return 0;
}
