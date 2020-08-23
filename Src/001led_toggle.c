/*
 * 001led_toggle.c
 *
 *      Author: henryco1
 */
#include "stm32f446xx.h"
#include <string.h>
#define BTN_PRESSED 0
/*
 * PUPD LED control
 */
void delay(int value) {
	for (uint32_t i=0; i<value; i++);
}

/*
 * Blinks LED2
 */
void blink_led(void) {
	GPIO_Handle_t gpioLED;
	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

//	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OPEN_DRAIN;
//	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULL_UP; // bc of open drain, pull resistor needed, but light would be dim
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	while(1) {
		GPIO_ToggleOutputPin(GPIOA, gpioLED.GPIO_PinConfig.GPIO_PinNumber);
		delay(500000);
	}
}

void button_toggle_led(void) {
	GPIO_Handle_t button_gpio;
	GPIO_Handle_t led_gpio;

	memset(&button_gpio, 0 , sizeof(GPIO_Handle_t));
	memset(&led_gpio, 0 , sizeof(GPIO_Handle_t));

	button_gpio.pGPIOx = GPIOC;
	led_gpio.pGPIOx = GPIOA;

	button_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	button_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	button_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	button_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;

	led_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	led_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	led_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	led_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	led_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&led_gpio);
	GPIO_Init(&button_gpio);

	while(1) {
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED) {
			delay(500000/2);
			GPIO_ToggleOutputPin(GPIOA, led_gpio.GPIO_PinConfig.GPIO_PinNumber);
		}
	}
}

int main(void) {
	button_toggle_led();
//	blink_led();
	return 0;
}

void EXTI0_IRQHandler(void) {
	GPIO_IRQHandling(0);
}

