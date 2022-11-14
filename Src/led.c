/*
 * led.c
 *
 *  Created on: Nov 14, 2022
 *      Author: NHHanh
 */

#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0; i < 50000; i++);
}

int main(void){
	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_MAX_SPEED_10MHZ;
	gpioLed.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
		delay();
	}
	return 0;
}
