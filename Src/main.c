/*
 * button_interupt.c
 *
 *  Created on: Oct 9, 2022
 *      Author: NHHanh
 */

#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

void delay(void){
	for(uint8_t i = 0; i < 250000; i++);
}


int main(void){
	GPIO_Handle_t gpioLed;
	GPIO_Handle_t gpioButton;
	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioButton, 0, sizeof(gpioButton));

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_MAX_SPEED_10MHZ;
	gpioLed.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioLed);

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinIRQ_TRIG = GPIO_MODE_IT_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_IN_PUPD;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

	//IRQ configuration
	GPIO_IRQPriConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIORITY_0);
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);



	return 0;
}
void EXTI0_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
}
