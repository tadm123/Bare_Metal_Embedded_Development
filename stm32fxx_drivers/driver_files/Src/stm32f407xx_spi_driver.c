/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: May 17, 2024
 *      Author: danielramirez
 */



#include "stm32f407xx_spi_driver.h"



//helper function (private) so no need to define it on .h
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*****************************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- Peripheral Clock settings
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (EnorDI == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}

	else 	//if EnorDi == DISABLE
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}

	}

}






/*****************************************************************************
 * @fn			- SPI_Init
 *
 * @brief		- Initializes SPI register
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)			//parameters: a pointer to the handle structure
{
	//	first lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// enable the peripheral clock (many times user forget, so we'll do it for them)
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)  //in full duplex there's two lines of communication (0)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) //in half fuplex there's one line of communication (1)
	{
		//bidi mode should be SET
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) //TXonly is full duplex, RXonlyis in bit 10
	{
		//bidi should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		//bidi should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. configure the clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;


	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;


	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = tempreg;
}




/*****************************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- Deinitializes SPI register
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)		// Deinitialize
{

}

//This will read the flags from the status register (SR), returning true or false
/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*****************************************************************************
 * @fn			- SPI_SendData
 *
 * @brief		- Data Send. this function call will wait until all bytes are transmitted. Number of bytes transmitted is indicates by the length (Len)
 * 				  Also called "Blocking call", because until all the bytes has been transferred this function will block and not return
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		- This is a blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)		//while Length (bytes) of data is > 0 , it means that there's still data to be transmitted, exits function when Len == 0 (data has finished transmitting)
	{
		//1. wait until TXE is set
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1. load the data into the DR (data register)
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;		//decrease two times because you just sent out 2 bytes of data
			Len--;
			(uint16_t*)pTxBuffer++;		//point to the next data item
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*****************************************************************************
 * @fn			- SPI_ReceiveData
 *
 * @brief		- Receives Data
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- void
 *
 * @Note		- none
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)		//while Length (bytes) of data is > 0 , it means that there's still data to be transmitted, exits function when Len == 0 (data has finished transmitting)
	{
		//1. wait until RXNE is set
		while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1. load the data from the DR (data register) to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;		//decrease two times because you just sent out 2 bytes of data
			Len--;
			(uint16_t*)pRxBuffer++;		//point to the next data item
		}
		else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}




/*****************************************************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		-
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}



/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}



/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}



/*****************************************************************************
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- IRQ interrupt configuration
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- void
 *
 * @Note		- none
 *
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI)
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
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- IRQ priority configuration
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;


	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	(*NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << (8 * iprx_section) );

}









/*****************************************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief		- ISR handling
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	// find which event caused the interrupt
	uint8_t temp1, temp2;

	//first lets check for TXE (if SPI_SR_TXE is set, temp1 will be set, if SPI_SR_TXE is 0, temp1 will be 0
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_CR2_TXEIE);




	if (temp1 && temp2)
	{
		//handle TXE flag
		spi_txe_interrupt_handle();
	}

	//check for RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
		{
			//handle RXNE
			spi_rxne_interrupt_handle();
		}

	//check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
		{
			//handle TXE
			spi_ovr_err_interrupt_handle();
		}


}


//helper function (private) implementations
//handle TXE flag
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. check the DFF bit in CR1
	if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		//16 bit DFF
		//1. load the data into the DR (data register)
		pSPIHandle->pSPIx->DR =  *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;		//point to the next data item
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =  *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		//If TxLen is zero, close the SPI transmission and inform the application that
		//Tx is over.

		//this prevents interrupts from setting up of txe flag
		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT); //inform application
	}
}


//handle RXNE flag
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. check the DFF bit in CR1
		if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1. load the data into the DR (data register)
			pSPIHandle->pSPIx->DR =  *((uint16_t*)pSPIHandle->pRxBuffer);
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			(uint16_t*)pSPIHandle->pRxBuffer--;		//point to the next data item
		}
		else
		{
			//8 bit DFF
			pSPIHandle->pSPIx->DR =  *pSPIHandle->pRxBuffer;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}

		if(!pSPIHandle->RxLen)
		{
			//reception is complete
			//let's turn off the rxnie interrupt
			pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
			pSPIHandle->pRxBuffer = NULL;
			pSPIHandle->RxLen = 0;
			pSPIHandle->RxState = SPI_READY;

			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT); //inform applicaiton
		}
}


//Handle OVR flag
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}



void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	//1. Save the Tx buffer address and Len information in some global variable (let's save them inside our SPI_Handle_t struct)
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other code can take over
	//   same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	//4. Data transmission will be handled by the ISR code (will be implemented later)

	return state;
}


void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX)
	//1. Save the Tx buffer address and Len information in some global variable (let's save them inside our SPI_Handle_t struct)
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other code can take over
	//   same SPI peripheral until transmission is over
	pSPIHandle->RxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	//4. Data transmission will be handled by the ISR code (will be implemented later)

	return state;
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//this is a weak implementation, the application may override this function
}
