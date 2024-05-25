/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 5, 2024
 *      Author: danielramirez
 */


#include "stm32f407xx_gpio_drive.h"




// defining my APIs functions


/*****************************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base of the gpio peripheral
 * @param[in]	- ENABLE or DISABLE macros
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)	//parameters: a pointer to the base address of the GPIO, tell whether it's enable or disable
{
	if (EnorDI == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOIPCLK_EN();
		}
	}
	else 	//if EnorDi == DISABLE
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOIPCLK_DI();
		}
	}

}




/*
 * Init and De Init
 */
// Initiaize the GPIO port or pin
/*****************************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)			//parameters: a pointer to the handle structure
{
	//1. configure the mode of GPIO pin
	uint32_t temp = 0;	//temporary register


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)	//accessing non-interrupt modes
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	//clearing bitfield (3 decimal is 11, and negation is 00)
		pGPIOHandle->pGIOx->MODER |= temp;	//setting the base MODER register to the value in temp


	}
	else 		//accessing interrupt modes
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			// set the corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			// set the corresponding RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the FTSR and RTSR
			// set both the bits corresponding with FTSR and RTSR bits
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}

	temp = 0;			//reitinitalize temp to 0
	//2. configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGIOx->OSPEEDR |= temp;	//setting

	temp = 0;


	//3. configure the pull-up pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGIOx->PUPDR |= temp;	//setting

	temp = 0;

	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
	}
}


/*****************************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base of the gpio peripheral
 * @param[in]	- ENABLE or DISABLE macros
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)		// Deinitialize
{
	if (pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
	}
}

/*
 * Data Read and Write
 */
/*****************************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		-
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- 1 or 0
 *
 * @Note		- none
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)			  // parameters: base address of the gpio peripheral, pin number. Return: either 0 or 1, so uint8 is fine
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);     //I only care about the LSB, so I mask all other bit positions with 0

	return value;
}





/*****************************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		-
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- 1 or 0
 *
 * @Note		- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)								  // parameters: base addres of gpio port. Return: content of input data register
{
	uint16_t value;
	value = (uint_16_)pGPIOx->IDR;

	return value;
}

/*****************************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)  // parameters: base address of the gpio peripheral, pin number, number either 0 or 1
{
	if (Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*****************************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- Writes to output port
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*****************************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- Toggles our pin
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) //toggle our pinNumber
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************************
 * @fn			- GPIO_IRQInterruptConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI)
{
	if (EnorDI == ENABLE)	// ISER (set enable registers)
	{
		if (IRQNumber <= 31)
		{
			//program ISERO register
			*NVIC_ISERO |= (1 << IRQNumber);

		}
		else if( (IRQNumber > 31) && (IRQNumber < 64) ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISERO |= (1 << (IRQNumber %32) );
		}
		else if( (IRQNumber >= 64) && (IRQNumber < 96) ) //64 to 95
		{
			//program ISER2 register
			*NVIC_ISERO |= (1 << (IRQNumber %64) );
		}
	}
	else 					// ICER (clear enable registers)
	{
		if (IRQNumber <= 31)
		{
			//program ISERO register
			*NVIC_ICERO |= (1 << IRQNumber);

		}
		else if( (IRQNumber > 31) && (IRQNumber < 64) ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ICERO |= (1 << (IRQNumber %32) );
		}
		else if( (IRQNumber >= 64) && (IRQNumber < 96) ) //64 to 95
		{
			//program ISER2 register
			*NVIC_ICERO |= (1 << (IRQNumber %64) );
		}
	}
}




/*****************************************************************************
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		-
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- 1 or 0
 *
 * @Note		- none
 *
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
		//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;


	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	(*NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << (8 * iprx_section) );

}



/*
 *
 */
/*****************************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- Note: This will override the existing IRQHandling on startup, whith the pinNumber that you specified.
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 */

void GPIO_IRQHandling(uint8_t PinNumber)		//When interrupt triggers user application can call this API function to process that interrupt
{
	//clear the exti pr register corresponding to the pin number
	if( EXTI->PR & (1 << PinNumber) )
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}



