/*
 * 015uart_tx.c
 *
 *  Created on: May 24, 2024
 *      Author: danielramirez
 */

#include <stdio.h>
#include "stm32f407xx.h"
#include <string.h>

char msg[1024] = "UART Tx testing ... \n\r";

USART_Handle_t USART2_Handle;


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void USART2_init(void)
{
	USART2_Handle.pUSARTx = USART2;		//base address

	USART2_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2_Handle.USART_Config.USART_Mode = USART_MODE_TX;
	USART2_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_Handle.USART_Config.USART_ParityControl = USART_WORDLEN_8BITS;
	USART2_Handle.USART_Config.USART_WordLength = USART_PARITY_DISABLE;

	USART_Init(&USART2_Handle);
}


void USART2_GPIOInit(void)
{
	GPIO_Handle_t USART_GPIOS;

	//configuring our GPIO settings for our for LED
	USART_GPIOS.pGPIOx = GPIOA;

	USART_GPIOS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART_GPIOS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; //can be anything
	USART_GPIOS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART_GPIOS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU;
	USART_GPIOS.GPIO_PinConfig.GPIO_PinAltFunMode = 7

	//USART2 Tx
	USART_GPIOS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USART_GPIOS);


	//USART2 Rx
	USART_GPIOS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;

	GPIO_Init(&USART_GPIOS);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}


int main (void)
{
	USART2_GPIOInit();

	USART2_Init();

	USART_PeripheralControl(USART2, ENABLE);


	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//delay to avoid debouncing
		delay();

		USART_SendData(&USART2_Handle, (uint8_t)*msg.strlen(msg));
	}

	return 0;
}
