/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: May 17, 2024
 *      Author: danielramirez
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral (user configures these on their program)
 */

typedef struct
{
	uint8_t SPI_DeviceMode;				/*!< @SPI_DeviceMode >*/
	uint8_t SPI_BusConfig;				/*!< @SPI_BusConfig >*/
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;



/*
 * SPI Application states
 */

#define SPI_READY			0
#define SPI_BUSY_IN RX		1
#define SPI_BUSY_IN_TX		2


/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT 		2
#define SPI_EVENT_OVR_ERR 		3
#define SPI_EVENT_CRC_ERR 		3

/*
 * Handle structure for SPIx peripherals
 */

typedef struct
{
	SPI_RegDef_t	*pSPIx;			 /*!< This holds the base address of SPIx (x:0,1,2) peripherals >*/
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint8_t			TxLen;
	uint8_t			RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;



/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0



/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					0		// Full duplex
#define SPI_BUS_CONFIG_HD					1		// Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		2		// Simplex receive only (simplex transmit only is full duplex)


/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2				0	//000
#define SPI_SCLK_SPEED_DIV4				1	//001
#define SPI_SCLK_SPEED_DIV8				2	//010
#define SPI_SCLK_SPEED_DIV16			3	//011
#define SPI_SCLK_SPEED_DIV32			4	//100
#define SPI_SCLK_SPEED_DIV64			5	//101
#define SPI_SCLK_SPEED_DIV128			6	//110
#define SPI_SCLK_SPEED_DIV256			7	//111


/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1




/*
 *@SPI_CPOL
 */

#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0




/*
 *@SPI_CPHA
 */

#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0




/*
 * @SPI_SSM
 */
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0				//default




/*
 * SPI related status flag definitions
 */

#define SPI_TXE_FLAG 		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG       (1 << SPI_SR_BSY)



/*************************************************************************************************
 * 						For more information about the APIs check the function definitions
**************************************************************************************************/


// defining my SPI functions



/*
 * Peripheral Clock settings
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);			//parameters: a pointer to the handle structure
void SPI_DeInit(SPI_RegDef_t *pSPIx);		// Deinitialize


/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other Peripheral Control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);






#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
