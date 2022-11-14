/*
 * STM32f103xx.h
 *
 *  Created on: Oct 2, 2022
 *      Author: NHHanh
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>
#include <stdbool.h>

#define _vo volatile

/**
 * ARM Cortex-Mx Processor NVIC ISERx register addresses
 * */
#define NVIC_ISER_0                 ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER_1                 ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER_2                 ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER_3                 ((volatile uint32_t *)0xE000E10C)
#define NVIC_ISER_4                 ((volatile uint32_t *)0xE000E110)
#define NVIC_ISER_5                 ((volatile uint32_t *)0xE000E114)
#define NVIC_ISER_6                 ((volatile uint32_t *)0xE000E118)
#define NVIC_ISER_7                 ((volatile uint32_t *)0xE000E11C)

/**
 * ARM Cortex-Mx Processor NVIC ICERx register addresses
 * */
#define NVIC_ICER_0                 ((volatile uint32_t *) 0XE000E180)
#define NVIC_ICER_1                 ((volatile uint32_t *) 0XE000E184)
#define NVIC_ICER_2                 ((volatile uint32_t *) 0XE000E188)
#define NVIC_ICER_3                 ((volatile uint32_t *) 0XE000E18C)
#define NVIC_ICER_4                 ((volatile uint32_t *) 0XE000E190)
#define NVIC_ICER_5                 ((volatile uint32_t *) 0XE000E194)
#define NVIC_ICER_6                 ((volatile uint32_t *) 0XE000E198)
#define NVIC_ICER_7                 ((volatile uint32_t *) 0XE000E19C)

/**
 * ARM Cortex-Mx Processor NVIC PR register addresses
 * */
#define NVIC_IPR_BASE_ADDR          ((volatile uint32_t *) 0xE000E400)

/**
 * Arm Cortex-Mx Processor number of priority bits implemented in Interrupt Priority Register
 * */
#define NO_IPR_BITS_IMPLEMENTED     4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDR				0x08000000U
#define SRAM1_BASE_ADDR				0x20000000U
#define ROM_BASE_ADDR				0x1FFFF000U		//system memory
#define SRAM						SRAM1_BASE_ADDR

/*
 * AHB and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE					0x40000000U
#define APB1_PERIPH_BASE_ADDR		PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR		0x40010000U
#define AHB_PERIPH_BASE_ADDR		0x40018000U

/*
 * Base addresses of peripherals which are hanging on APB1
 */

#define TIM2_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x0C00)
#define TIM6_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x1400)
#define TIM12_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x1800)
#define TIM13_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x1C00)
#define TIM14_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x2000)

#define RTC_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x3000)

#define SPI2_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x5000)

#define I2C1_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x5800)

#define USB_FS_REG_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5C00)

#define CAN_SRAM_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x6000)
#define CAN2_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x6800)
#define CAN1_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x6400)

#define BKP_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x6C00)

#define PWR_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x7000)
#define DAC_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x7400)


/*
 * Base addresses of peripherals which are hanging on APB2
 */

#define AFIO_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x0000)
#define EXTI_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x0400)

#define GPIOA_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x0800)
#define GPIOB_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOC_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x1000)
#define GPIOD_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x1400)
#define GPIOE_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x1800)
#define GPIOF_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x1C00)
#define GPIOG_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x2000)

#define ADC1_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x2400)
#define ADC2_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x2800)

#define TIM1_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x2C00)
#define TIM8_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3400)
#define TIM9_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x4C00)
#define TIM10_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x5000)
#define TIM11_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x5400)

#define SPI1_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3000)

#define USART1_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x3800)

#define ADC3_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3C00)




/*
 * Base addresses of peripherals which are hanging on AHB
 */

#define SDIO_BASE_ADDR				(AHB_PERIPH_BASE_ADDR + 0x0000)

#define DMA1_BASE_ADDR				(AHB_PERIPH_BASE_ADDR + 0x8000)
#define DMA2_BASE_ADDR				(AHB_PERIPH_BASE_ADDR + 0x8400)

#define RCC_BASE_ADDR				(AHB_PERIPH_BASE_ADDR + 0x9000)

#define FLASH_MEM_INTF_BASE_ADDR	(AHB_PERIPH_BASE_ADDR + 0xA000)
#define CRC_BASE_ADDR				(AHB_PERIPH_BASE_ADDR + 0xB000)

#define ETHERNET_BASE_ADDR			(AHB_PERIPH_BASE_ADDR + 0x10000)

#define USB_OTG_FS_BASE_ADDR		(AHB_PERIPH_BASE_ADDR + 0xFFE8000)
#define FSMC_BASE_ADDR				(AHB_PERIPH_BASE_ADDR + 0x5FFE8000)

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	_vo uint32_t CR[2];				// Port configuration register low																Address offset:0x00 - 0x04
	_vo uint32_t IDR;				// Port input data register																		Address offset:0x08
	_vo uint32_t ODR;				// Port input data register																		Address offset:0x0C
	_vo uint32_t BSRR;				// Port bit set/reset register																	Address offset:0x10
	_vo uint32_t BRR;				// Port bit reset register																		Address offset:0x14
	_vo uint32_t LCKR;				// Port configuration lock register																Address offset:0x18
}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */

typedef struct
{
	_vo uint32_t CR;				// Clock control register																		Address offset:0x00
	_vo uint32_t CFGR;				// Clock configuration register																	Address offset:0x04
	_vo uint32_t CIR;				// Clock interrupt register																		Address offset:0x08
	_vo uint32_t APB2RSTR;			// APB2 peripheral reset register																Address offset:0x0C
	_vo uint32_t APB1RSTR;			// APB1 peripheral reset register																Address offset:0x10
	_vo uint32_t AHBENR;			// AHB Peripheral Clock enable register															Address offset:0x14
	_vo uint32_t APB2ENR;			// APB2 peripheral clock enable register														Address offset:0x18
	_vo uint32_t APB1ENR;			// APB1 peripheral clock enable register														Address offset:0x1C
	_vo uint32_t BDCR;				// Backup domain control register																Address offset:0x20
	_vo uint32_t CSR;				// Control/status register																		Address offset:0x24
	_vo uint32_t AHBRSTR;			// AHB peripheral clock reset register															Address offset:0x28
	_vo uint32_t CFGR2;				// Clock configuration register2																Address offset:0x3C

}RCC_RegDef_t;

typedef struct
{
	_vo uint32_t IMR;			// Interrupt mask register																		Address offset:0x00
	_vo uint32_t EMR;			// Event mask registerPort input data register													Address offset:0x04
	_vo uint32_t RTSR;			// Rising trigger selection register															Address offset:0x08
	_vo uint32_t FTSR;			// Falling trigger selection register															Address offset:0x0C
	_vo uint32_t SWIER;			// Software interrupt event register															Address offset:0x10
	_vo uint32_t PR;			// Pending register																				Address offset:0x14
}EXTI_RegDef_t;

typedef struct
{
	_vo uint32_t EVCR;			// Event control register																		Address offset:0x00
	_vo uint32_t MAPR;			// AF remap and debug I/O configuration register												Address offset:0x04
	_vo uint32_t EXTICR[4];		// Rising trigger selection register															Address offset:0x08
	_vo uint32_t MAPR2;			// AF remap and debug I/O configuration register2												Address offset:0x1C
}AFIO_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */

typedef struct
{
	_vo uint32_t CR1;				// I2C Control register	1																	Address offset:0x00
	_vo uint32_t CR2;				// I2C Control register 2 																	Address offset:0x04
	_vo uint32_t OAR1;				// I2C Own address register 1																Address offset:0x08
	_vo uint32_t OAR2;				// I2C Own address register 2																Address offset:0x0C
	_vo uint32_t DR;				// I2C Data register																		Address offset:0x10
	_vo uint32_t SR1;				// I2C Status register 1																	Address offset:0x14
	_vo uint32_t SR2;				// I2C Status register 2																	Address offset:0x18
	_vo uint32_t CCR;				// I2C Clock control register																Address offset:0x1C
	_vo uint32_t TRISE;				// I2C TRISE register																		Address offset:0x20
}I2C_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF 						((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG 						((GPIO_RegDef_t*)GPIOG_BASE_ADDR)

/*
 * RCC definitions
 */
#define RCC 						((RCC_RegDef_t*)RCC_BASE_ADDR)

/*
 * Interrupt definitions
 */
#define EXTI						((EXTI_RegDef_t*)EXTI_BASE_ADDR)

/*
 * AFIO definitions
 */
#define AFIO						((AFIO_RegDef_t*)AFIO_BASE_ADDR)

/*
 * I2C definitions
 */
#define I2C1						((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2						((I2C_RegDef_t*)I2C2_BASE_ADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			(RCC -> APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()			(RCC -> APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()			(RCC -> APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()			(RCC -> APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()			(RCC -> APB2ENR |= (1 << 6))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 6))

/*
 * Clock Enable Macros for AFIO
 */

#define AFIO_PCLK_EN()			(RCC -> APB2ENR |= (1 << 0))

/*
 * Clock Disnable Macros for AFIO
 */

#define AFIO_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 0))

/*
 * Clock Enable Macros for I2C
 */

#define I2C1_PCLK_EN()			(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 22))

/*
 * Clock Disnable Macros for I2C
 */

#define I2C1_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 22))

/*
 * GPIOx peripheral reset macros
 * */
#define GPIOA_REG_RST()       do{(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));} while(false)
#define GPIOB_REG_RST()       do{(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));} while(false)
#define GPIOC_REG_RST()       do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while(false)
#define GPIOD_REG_RST()       do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while(false)
#define GPIOE_REG_RST()       do{(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while(false)

/*
 * returns port code for given GPIOx base address
 */

#define GPIO_BASE_ADDR_TO_CODE(x)	   ((x == GPIOA) ? 0b0000 : \
										(x == GPIOB) ? 0b0001 : \
										(x == GPIOC) ? 0b0010 : \
										(x == GPIOD) ? 0b0011 : \
										(x == GPIOE) ? 0b0100 : 0)

/**
 * @IRQ_NUMBERS
 * IRQ(Interrupt Request) numbers
 * */
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

/**
 * @IRQ_PRIORITES
 * IRQ(Interrupt Request) priorities
 * */
#define NVIC_IRQ_PRIORITY_0     0
#define NVIC_IRQ_PRIORITY_1     1
#define NVIC_IRQ_PRIORITY_2     2
#define NVIC_IRQ_PRIORITY_3     3
#define NVIC_IRQ_PRIORITY_4     4
#define NVIC_IRQ_PRIORITY_5     5
#define NVIC_IRQ_PRIORITY_6     6
#define NVIC_IRQ_PRIORITY_7     7
#define NVIC_IRQ_PRIORITY_8     8
#define NVIC_IRQ_PRIORITY_9     9
#define NVIC_IRQ_PRIORITY_10    10
#define NVIC_IRQ_PRIORITY_11    11
#define NVIC_IRQ_PRIORITY_12    12
#define NVIC_IRQ_PRIORITY_13    13
#define NVIC_IRQ_PRIORITY_14    14
#define NVIC_IRQ_PRIORITY_15    15

/*******************************************
 * Bit position definitions of I2C peripheral
 *******************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_NONSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD71			1
#define I2C_OAR1_ADD98			8
#define I2C_OAR1_ADDMODE		15

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_SR2_CCR				0
#define I2C_SR2_DUTY			14
#define I2C_SR2_FS				15



//some generic marcos
#define ENABLE 					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				1
#define FLAG_RESET				0

#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_i2c_driver.h"

#endif /* INC_STM32F103XX_H_ */
