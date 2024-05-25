/*
 * 002ledbutton.c
 *
 *  Created on: May 16, 2024
 *      Author: danielramirez
 */

#include "stm32f407xx.h"


#define ENABLE 1
#define BTN_PRESSED ENABLE

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void)
{

	GPIO_Handle_t GPIOLed, GPIOBtn;

	//enabling the clock
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//initialize
	GPIO_Init(&GPIOLed);


	//configuring our GPIO settings for our for LED
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;


	//enabling the clock
	GPIO_PeriClockControl(GPIOA, ENABLE);

	//initialize
	GPIO_Init(&GPIOBtn);

	while(1)
	{

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}

	return 0;
}
