/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Oct 4, 2022
 *      Author: NHHanh
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * This is a configuration structure for a GPIO pin
 */

typedef struct
{
	 uint8_t GPIO_PinNumber;                 /** possible values from @GPIO_PIN_NUMBERS */
	 uint8_t GPIO_PinMode;                   /** possible values values from @GPIO_PIN_MODES */
	 uint8_t GPIO_PinCNF;                    /** possible values values from @GPIO_PIN_CNF */
	 uint8_t GPIO_PinIRQ_TRIG;               /** possible values values from @GPIO_PIN_IRQ */
	 uint8_t GPIO_PinPuPdControl;
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	//Pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t* pGPIOx;		//This holds the base address of the GPIO port to which the pin belong
	GPIO_PinConfig_t GPIO_PinConfig;		//This holds GPIO pin configuration settings
}GPIO_Handle_t;



/*
 * APIs suppoerted by this driver
 * For more information about the APIs check the function definitiond
 */

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);



/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_OUT_MAX_SPEED_2MHZ        0b10
#define GPIO_MODE_OUT_MAX_SPEED_10MHZ       0b01
#define GPIO_MODE_OUT_MAX_SPEED_50MHZ       0b11
#define GPIO_MODE_IN                        0b00


/*
 * @GPIO_PIN_IRQ
 */
#define GPIO_MODE_IT_FT                         0b100 // Interrupt trigger falling edge
#define GPIO_MODE_IT_RT                         0b101 // Interrupt trigger rising edge
#define GPIO_MODE_IT_RFT                        0b110 // Interrupt trigger falling and rising edge

/*
 * @GPIO_PIN_CNF
 */
#define GPIO_CNF_IN_A                       0b00 // Analog mode
#define GPIO_CNF_IN_F                       0b01 // Floating input (reset state)
#define GPIO_CNF_IN_PUPD                    0b10 // Input with pull-up / pull-down
#define GPIO_CNF_IN_RESERVED                0b11 // RESERVED

#define GPIO_CNF_OUT_PP                     0b00 // General purpose output push-pull
#define GPIO_CNF_OUT_OD                     0b01 // General purpose output Open-drain
#define GPIO_CNF_OUT_AF_PP                  0b10 // Alternate function output Push-pull
#define GPIO_CNF_OUT_AF_OD                  0b11 // Alternate function output Open-drain

#define GPIO_PU                             1 // Pull-up
#define GPIO_PD                             0 // Pull-down

/*
 * GPIO pin numbers
 */
#define GPIO_PIN_0                          0
#define GPIO_PIN_1                          1
#define GPIO_PIN_2                          2
#define GPIO_PIN_3                          3
#define GPIO_PIN_4                          4
#define GPIO_PIN_5                          5
#define GPIO_PIN_6                          6
#define GPIO_PIN_7                          7
#define GPIO_PIN_8                          8
#define GPIO_PIN_9                          9
#define GPIO_PIN_10                         10
#define GPIO_PIN_11                         11
#define GPIO_PIN_12                         12
#define GPIO_PIN_13                         13
#define GPIO_PIN_14                         14
#define GPIO_PIN_15                         15




#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
