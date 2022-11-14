/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Oct 4, 2022
 *      Author: NHHanh
 */

#include "stm32f103xx_gpio_driver.h"
#include <stdint.h>

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
    if (EnOrDi) {
        if (pGPIOx == GPIOA)
        	GPIOA_PCLK_EN();
        else if (pGPIOx == GPIOB)
        	GPIOB_PCLK_EN();
        else if (pGPIOx == GPIOC)
        	GPIOC_PCLK_EN();
        else if (pGPIOx == GPIOD)
        	GPIOD_PCLK_EN();
        else if (pGPIOx == GPIOE)
        	GPIOE_PCLK_EN();
    } else {
        if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
        else if (pGPIOx == GPIOB)
        	GPIOB_PCLK_DI();
        else if (pGPIOx == GPIOC)
        	GPIOC_PCLK_DI();
        else if (pGPIOx == GPIOD)
        	GPIOD_PCLK_DI();
        else if (pGPIOx == GPIOE)
        	GPIOE_PCLK_DI();
    }
}

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	GPIO_PeriClockControl(pGPIOHandle ->pGPIOx, ENABLE);

	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

	 //1. configure the mode of gpio pin

	 if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= 0b11){
		 pGPIOHandle -> pGPIOx->CR[temp1] &= ~(1 << (4*temp2));
		 pGPIOHandle -> pGPIOx->CR[temp1] |= pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (4*temp2);
	//2. configure cnf of gpio pin
		 pGPIOHandle -> pGPIOx->CR[temp1] &= ~(1 << (4*temp2 + 2));
		 pGPIOHandle -> pGPIOx->CR[temp1] |= pGPIOHandle -> GPIO_PinConfig.GPIO_PinCNF << (4*temp2 + 2);

	//3. configure pupd settings
	 if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_IN) {
		 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PU)
			 pGPIOHandle->pGPIOx->ODR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	 else pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	 if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN){
		 if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinIRQ_TRIG == GPIO_MODE_IT_FT){
			 //1. configure the FTSR
			 EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 EXTI -> RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinIRQ_TRIG == GPIO_MODE_IT_RT){
			 //2. configure the RTSR
			 EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 EXTI -> FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinIRQ_TRIG == GPIO_MODE_IT_RFT){
			 //3. configure both FTSR and RTSR
			 EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
	 }
	 //2. configure the GPIO port selection in AFIO_EXTICR
			uint8_t temp3 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp4 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle -> pGPIOx);
			AFIO_PCLK_EN();
			AFIO->EXTICR[temp3] = portcode << (temp4 * 4);

	 //3. enable the exti interrupt delivery using IMR
			EXTI -> IMR |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);

	 }


}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	 if (pGPIOx == GPIOA)
		 GPIOA_REG_RST();
	 else if (pGPIOx == GPIOB)
		 GPIOB_REG_RST();
	 else if (pGPIOx == GPIOC)
		 GPIOC_REG_RST();
	 else if (pGPIOx == GPIOD)
		 GPIOD_REG_RST();
	 else if (pGPIOx == GPIOE)
		 GPIOE_REG_RST();
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	return (pGPIOx -> IDR >> pinNumber) & 1;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	return pGPIOx -> IDR;

}
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if (value == GPIO_PIN_SET){
		pGPIOx -> ODR |= (1 << pinNumber);
	}else
		pGPIOx -> ODR &= ~(1 << pinNumber);
}
void WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx -> ODR = value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx -> ODR ^= (1 << pinNumber);

}

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER_0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63){
			*NVIC_ISER_1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96){
			*NVIC_ISER_2 |= (1 << (IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber <= 31){
				*NVIC_ICER_0 |= (1 << IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber <= 63){
				*NVIC_ICER_1 |= (1 << (IRQNumber % 32));
			}
			else if(IRQNumber > 63 && IRQNumber < 96){
				*NVIC_ICER_2 |= (1 << (IRQNumber % 64));
			}
	}

}

void GPIO_IRQPriConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (iprx_section * 8) + (8 - NO_IPR_BITS_IMPLEMENTED);
    *(NVIC_IPR_BASE_ADDR + iprx) |= IRQPriority << shift_amount;	//Su dung 4 bit cao de cau hinh
}

void GPIO_IRQHandling(uint8_t pinNumber){
    // clear the EXTI PR register corresponding to the pin
    if (EXTI->PR & (1 << pinNumber))
    	EXTI->PR |= (1 << pinNumber); // clear

}










