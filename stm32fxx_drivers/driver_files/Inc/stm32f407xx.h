/*
 * stm32f407xx.h
 *
 *  Created on: May 4, 2024
 *      Author: Daniel Ramirez
 */


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>		//for the NULL variable

#define __vo volatile			// Since many registers in memory are highly volatile
#define __weak __attribute__((weak))

/*************************** START: Processor Specific Details ***************************
 *
 *
 * ARM Cortex Mx Processor NVIC ISERx (Interrupt Set-enable) register addresses
 */

#define NVIC_ISER0				( (_vo uint31_t*)0xE000E100 )
#define NVIC_ISER1				( (_vo uint31_t*)0xE000E104 )
#define NVIC_ISER2				( (_vo uint31_t*)0xE000E108 )
#define NVIC_ISER3				( (_vo uint31_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx (Interrupt Clear-enable) register addresses
 */

#define NVIC_ICER0				( (_vo uint31_t*)0xE000E100 )
#define NVIC_ICER1				( (_vo uint31_t*)0xE000E104 )
#define NVIC_ICER2				( (_vo uint31_t*)0xE000E108 )
#define NVIC_ICER3				( (_vo uint31_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx (Interrupt Priority Registers) register addresses
 */
#define NVIC_PR_BASE_ADDR		( (_vo uint32_t*)0xE000E400 )


/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4



/* base address of Flash and SRAM memories */

#define FLASH_BASEADDR						0x08000000U 	/* Flash base address */
#define SRAM1_BASEADDR						0x20000000U 	/* SRAM1 base address */
#define SRAM2_BASEADDR						0x20001C00U 	/* SRAM2 base address */
#define ROM_BASEADDR						0x1FFF0000U		/* ROM base address */
#define SRAM								SRAM1_BASEADDR 	/* Flash base address */



/* AHBx and APBx Bus peripheral base addresses */

#define PERIPH_BASEADDR						0x40000000U 	/* Peripheral base address */
#define APB1PERIPH_BASEADDR					PERIPH_BASE 	/* APB1 peripheral is the same as base address */
#define APB2PERIPH_BASEADDR					0x40010000U 	/* APB1 base address */
#define AHB1PERIPH_BASEADDR					0x40020000U		/* AHB1 base address */
#define AHB2PERIPH_BASEADDR					0x50000000U 	/* AHB2 base address */



/* Base addresses of peripherals which are hanging into AHB1 bus */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000) 	   /* (AHB1PERIPH_BASEADDR + OFFSET) */
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0004)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0008)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)


/* Base addresses of peripherals which are hanging into APB1 bus */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)


/* Base addresses of peripherals which are hanging into APB2 bus */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)



/************************ peripheral register definition  structures ********************/
/*
 * Note: Registers of a peripheral are specific to MCU
 */


typedef struct
{
	__vo uint32_t MODER;		// GPIO port mode register     			    Address offset: 0x00
	__vo uint32_t OTYPER;		// GPIO port output type register     		Address offset: 0x04
	__vo uint32_t OSPEEDR;		// GPIO port output speed register     		Address offset: 0x08
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register     Address offset: 0x0C
	__vo uint32_t IDR;			// GPIO port input data register     		Address offset: 0x10
	__vo uint32_t ODR;			// GPIO port output data register     		Address offset: 0x14
	__vo uint32_t BSRR;			// GPIO port bit set/reset register    		Address offset: 0x18
	__vo uint32_t LCKR;			// GPIO port configuration lock register    Address offset: 0x1C
	__vo uint32_t AFR[2];		// GPIO alternate function (low and high)	Address offset: 0x20
								// Note: We have two Alternate function peripherals (AFRL and AFRH). We can define them separately, or we can do AFR[0] meaning low, and AFR[1] meaning high
}GPIO_RegDef_t;



/*
 * peripheral registers definition structures for RCC
 */

typedef struct
{
	__vo uint32_t CR;			// RCC clock control register     			    Address offset: 0x00
	__vo uint32_t PLLCFGR;		// RCC PLL configuration register     			Address offset: 0x04
	__vo uint32_t CFGR;			// RCC clock configuration register      		Address offset: 0x08
	__vo uint32_t CIR;			// RCC clock interrupt register     			Address offset: 0x0C
	__vo uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register      		Address offset: 0x10
	__vo uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register      		Address offset: 0x14
	__vo uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register    		Address offset: 0x18
	uint32_t 	  RESERVED0;	// Reserved register 0x1c
	__vo uint32_t APB1RSTR;		// RCC APB1 peripheral reset register    		Address offset: 0x20
	__vo uint32_t APB2RSTR;		// RCC APB2 peripheral reset register    		Address offset: 0x24
	uint32_t 	  RESERVED1[2];	// Reserved registers 0x28-0x2c
	__vo uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register    Address offset: 0x30
	__vo uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register    Address offset: 0x34
	__vo uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register    Address offset: 0x38
	uint32_t 	  RESERVED2;	// Reserved registers 0x3C
	__vo uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register    Address offset: 0x40
	__vo uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register    Address offset: 0x44
	uint32_t 	  RESERVED3[2];	// Reserved registers 0x48-0x4C
	__vo uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable register    Address offset: 0x50
	__vo uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable register    Address offset: 0x54
	__vo uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable register    Address offset: 0x58
	uint32_t 	  RESERVED4;	// Reserved registers 0x5C
	__vo uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register    		Address offset: 0x60
	__vo uint32_t APB2LPENR;	// RCC APB2 peripheral clock enable in low power mode register    		Address offset: 0x64
	uint32_t 	  RESERVED5[2];	// Reserved registers 0x68-0x6C
	__vo uint32_t RCC_BDCR;		// RCC Backup domain control register    		Address offset: 0x70

}RCC_Regdef_t;



/*
 * peripheral registers definition structures for EXTI
 */


typedef struct
{
	__vo uint32_t IMR;		// Interrupt mask register (EXTI_IMR)  			    Address offset: 0x00
	__vo uint32_t EMR;		// Event mask register (EXTI_EMR) 					Address offset: 0x04
	__vo uint32_t RTSR;		// Rising trigger selection register (EXTI_RTSR)   	Address offset: 0x08
	__vo uint32_t FTSR;		// Falling trigger selection register (EXTI_FTSR)   Address offset: 0x0C
	__vo uint32_t SWIER;	// Software interrupt event register (EXTI_SWIER)   Address offset: 0x10
	__vo uint32_t PR;		// Pending register (EXTI_PR)    					Address offset: 0x14

}EXTI_RegDef_t;





/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;		// SYSCFG memory remap register (SYSCFG_MEMRMP)   			    Address offset: 0x00
	__vo uint32_t PMC;			// SYSCFG peripheral mode configuration register (SYSCFG_PMC) 	Address offset: 0x04
	__vo uint32_t EXTICR[4];	// SYSCFG external interrupt configuration registers   		    Address offset: 0x08-0x14
	uint32_t RESERVED[2];		// RESEVERD   													0x0x18-0x1C
	__vo uint32_t CMPCR;		// Compensation cell control register (SYSCFG_CMPCR)   			Address offset: 0x20
	uint32_t RESERVED[2];		// RESEVERD  				 									0x24-0x28
	__vo uint32_t CFGR;			// Pending register (EXTI_PR)    								Address offset: 0x2C

}SYSCFG_RegDef_t;




/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;		    // SPI control register 1 (SPI_CR1)   			   			Address offset: 0x00
	__vo uint32_t CR2;			// SPI control register 2 (SPI_CR2) 				 		Address offset: 0x04
	__vo uint32_t SR;	  		// SPI status register (SPI_SR)   		    				Address offset: 0x08
	__vo uint32_t DR;			// SPI data register (SPI_DR)   							Address offset: 0x0c
	__vo uint32_t CFGR;			// SPI CRC polynomial register (SPI_CRCPR)   				Address offset: 0x10
	__vo uint32_t RXCRCR;		// SPI RX CRC register (SPI_RXCRCR)     					Address offset: 0x14
	__vo uint32_t TXCRCR;		// SPI TX CRC register (SPI_TXCRCR)    						Address offset: 0x18
	__vo uint32_t I2SCFGR;		// SPI_I^2S configuration register (SPI_I2SCFGR)    		Address offset: 0x1C
	__vo uint32_t I2SPR;		// SPI_I^2S prescaler register (SPI_I2SPR)    				Address offset: 0x20
}SPI_RegDef_t;





/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;		    // I2C Control register 1 (I2C_CR1)  		Address offset: 0x00
	__vo uint32_t CR2;			// I2C Control register 1 (I2C_CR1)   		Address offset: 0x04
	__vo uint32_t OAR1;	  		// SPI status register (SPI_SR)   		    Address offset: 0x08
	__vo uint32_t OAR2;			// I2C Own address register 1 (I2C_OAR1)   	Address offset: 0x0c
	__vo uint32_t DR;			// I2C Data register (I2C_DR)  				Address offset: 0x10
	__vo uint32_t SR1;		// I2C Status register 1 (I2C_SR1)     			Address offset: 0x14
	__vo uint32_t SR2;		// I2C Status register 2 (I2C_SR2)  			Address offset: 0x18
	__vo uint32_t CCR;		// I2C Clock control register (I2C_CCR)    		Address offset: 0x1C
	__vo uint32_t TRISE;		// I2C TRISE register (I2C_TRISE)    		Address offset: 0x20
	__vo uint32_t FLTR;		// I2C FLTR register (I2C_FLTR)    				Address offset: 0x24
}I2C_RegDef_t;





/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR1;		    // Status register (USART_SR)		  		Address offset: 0x00
	__vo uint32_t DR;			// Data register (USART_DR			   		Address offset: 0x04
	__vo uint32_t BBR;	  		// Baud rate register (USART_BRR)   	    Address offset: 0x08
	__vo uint32_t CR1;			// Control register 1 (USART_CR1)   		Address offset: 0x0c
	__vo uint32_t CR2;			// Control register 2 (USART_CR2)  			Address offset: 0x10
	__vo uint32_t CR3;			// Control register 3 (USART_CR3)     		Address offset: 0x14
	__vo uint32_t GTPR;			// Guard time and prescaler register (USART_GTPR) 		Address offset: 0x18

}USART_RegDef_t;







/*
 * peripheral definitions (Peripherals base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOi_BASEADDR)



#define RCC			    ((RCC_Regdef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



#define SPI1 			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)



#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)


#define USART1			((I2C_RegDef_t*)USART1_BASEADDR)
#define USART2			((I2C_RegDef_t*)USART2_BASEADDR)
#define USART3			((I2C_RegDef_t*)USART3_BASEADDR)
#define UART4			((I2C_RegDef_t*)UART4_BASEADDR)
#define UART5			((I2C_RegDef_t*)UART5_BASEADDR)
#define USART6			((I2C_RegDef_t*)USART6_BASEADDR)
#define UART7			((I2C_RegDef_t*)UART7_BASEADDR)
#define UART8			((I2C_RegDef_t*)UART8_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )      // In long form: GPIO_PERI_CLOCK_ENABLE(). This shifts 0 bits to the left and sets it
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )	  // shifts 1 bit and set it
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()		( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()		( RCC->AHB1ENR |= (1 << 10))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )


/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= (1 << 13) )








/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) ) //make sure its correct
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5) )


#define UART4_PCLK_EN()		( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= (1 << 20) )
#define UART7_PCLK_EN()		( RCC->APB1ENR |= (1 << 30) )
#define UART8_PCLK_EN()		( RCC->APB1ENR |= (1 << 31) )



/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )




/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )      // This shifts 0 bits to the left and clears it
#define GPIOB_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )	  // shifts 1 bit and clears it
#define GPIOC_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 10))


/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )


/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 13) )





/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) ) //make sure its correct
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )

#define UART4_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 20) )
#define UART7_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 30) )
#define UART8_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 31) )



/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCCK_DI()	( RCC->APB2ENR &= ~(1 << 14) )


/*
 * MACROS to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0) );	(RCC->AHB1RSTR |= (1 << 0) ); }while(0)		//this is a way to use 2 or 3 statments in a C macro by using do while loops
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1) );	(RCC->AHB1RSTR |= (1 << 1) ); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2) );	(RCC->AHB1RSTR |= (1 << 2) ); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3) );	(RCC->AHB1RSTR |= (1 << 3) ); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4) );	(RCC->AHB1RSTR |= (1 << 4) ); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5) );	(RCC->AHB1RSTR |= (1 << 5) ); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6) );	(RCC->AHB1RSTR |= (1 << 6) ); }while(0)
#define GPIOJ_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7) );	(RCC->AHB1RSTR |= (1 << 7) ); }while(0)
#define GPIOJ_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8) );	(RCC->AHB1RSTR |= (1 << 8) ); }while(0)


/*
 * returns port code for given GPIOx base address
 *
 *
 * this macro returns a code (between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 : 0 )



/*
 * IRQ (Interrupt requests) numbers of SM32F407x MCU
 * NOTE:
 */

#define IRQ_NO_EXTI0	  6
#define IRQ_NO_EXTI1	  7
#define IRQ_NO_EXTI2	  8
#define IRQ_NO_EXTI3	  9
#define IRQ_NO_EXTI4	  10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40

#define IRQ_NO_SPI1		  35
#define IRQ_NO_SPI2		  36
#define IRQ_NO_SPI3		  51

#define IRQ_NO_I2C1_EV	  31
#define IRQ_NO_12C1_ER	  32
#define IRQ_NO_12C2_EV	  33
#define IRQ_NO_12C2_ER	  34
#define IRQ_NO_12C3_EV	  72
#define IRQ_NO_12C3_ER	  73




/*
 * macros for all possible priority levels
 */

#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI1	1
#define NVIC_IRQ_PRI2	2
#define NVIC_IRQ_PRI3	3
#define NVIC_IRQ_PRI4	4
#define NVIC_IRQ_PRI5	5
#define NVIC_IRQ_PRI6	6
#define NVIC_IRQ_PRI7	7
#define NVIC_IRQ_PRI8	8
#define NVIC_IRQ_PRI9	9
#define NVIC_IRQ_PRI10	10
#define NVIC_IRQ_PRI11	11
#define NVIC_IRQ_PRI12	12
#define NVIC_IRQ_PRI13	13
#define NVIC_IRQ_PRI14	14
#define NVIC_IRQ_PRI15	15





// some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET 		RESET
#define FLAG_SET 		SET


/***********************************************************************************************/
/* Bit position definition for SPI peripheral
/***********************************************************************************************/

//for SPI control register 1 (SPI_CR1)
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15



//for SPI control register 2 (SPI_CR2)

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7



//for SPI status register (SPI_SR)

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8



/***********************************************************************************************/
/* Bit position definition for I2C peripheral
/***********************************************************************************************/

//for I2C control register 1 (I2C_CR1)
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15


//for I2C control register 2 (I2C_CR2)
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12



//for I2C status register 1 (I2C_SR1)
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
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15



//for I2C status register 2 (I2C_SR2)
#define I2C_SR1_MSL				0
#define I2C_SR1_BUSY			1
#define I2C_SR1_TRA				2
#define I2C_SR1_GENCALL			4
#define I2C_SR1_SMBDEFAULT		5
#define I2C_SR1_SMBHOST			6
#define I2C_SR1_DUALF			7
#define I2C_SR1_PEC				8


//for I2C Clock control register (I2C_CCR)
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15




/***********************************************************************************************/
/* Bit position definition for USART peripheral
/***********************************************************************************************/

//for USART Control register 1 (USART_CR1)
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13


//for USART control register 2 (USART_C2)
#define USART_CR2_ADDR			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14


//for USART control register3 (USART_C2)
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11



//for USART Status register (USART_SR)
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9


#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
