/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 5, 2024
 *      Author: danielramirez
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"


/*
 * This is a Configuration structure for a GPIO pin (user configurates these on their program)
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			//holds pin number (from 0-15) /*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;			/*!< possible values from @GPIO_PIN_OPTYPE >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;



/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;				/*< This holds the base address of the GPIO port to which the pin belongs >*/

	GPIO_PinConfig_t GPIO_PinConfig;	/*< This holds GPIO pin configuration settings >*/
}GPIO_Handle_t;




/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN 		0			// Input (reset state), this decimal version of 00 (if I wanted hex, I'd put 0x0)
#define GPIO_MODE_OUT 		1			// General purpose output mode this is 01
#define GPIO_MODE_ALTFN 	2			// Alternate function mode, this is 10
#define GPIO_MODE_ANALOG 	3			// Analog mode, this is 11

#define GPIO_MODE_IT_FT		4			// defining GPIO Mode Falling edge trigger (IT = input) (FT = falling edge)
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6



/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP		0 			//push pull (reset state)
#define GPIO_OP_TYPE_OD		1			//open drain


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD		0			//no pull-up,pull-down
#define GPIO_PIN_PU			1			//pull up
#define GPIO__PIN_PD		2			//pull down


/*
 *@GPIO_PIN_OPTYPE
 */


/*************************************************************************************************
 * 											API supported by this driver
 * 						For more information about the APIs check the function definitions
**************************************************************************************************/


// defining my APIs functions


/*
 * Peripheral Clock setup
 */
// You're going to enable or disable for given GPIO base address
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);	//parameters: a pointer to the base address of the GPIO, tell whether it's enable or disable


/*
 * Init and De Init
 */
// Initiaize the GPIO port or pin
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);			//parameters: a pointer to the handle structure
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);		// Deinitialize

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);			  // parameters: base address of the gpio peripheral, pin number. Return: either 0 or 1, so uint8 is fine
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);								  // parameters: base addres of gpio port. Return: content of input data register
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);  // parameters: base address of the gpio peripheral, pin number, number either 0 or 1
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);		//When interrupt triggers user application can call this API function to process that interrupt






#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
