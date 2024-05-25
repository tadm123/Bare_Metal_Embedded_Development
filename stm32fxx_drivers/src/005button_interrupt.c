/*
 * 005button_interrupt.c
 *
 *  Created on: May 17, 2024
 *      Author: danielramirez
 */


#include <string.h>



void delay()
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for (uint32_t i = 0; i < 500000/2 ; i++);
}

int main(void)
{

	GPIO_Handle_t GPIOLed, GPIOBtn;
	memset( &GPIOLed, 0, sizeof(GPIOLed) );
	memset( &GPIOBtn, 0, sizeof(GPIOBtn) );


	//configuring our GPIO settings for our for LED
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;


	//enabling the clock
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//initialize
	GPIO_Init(&GPIOLed);




	//configuring our GPIO settings for our for LED
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;


	//enabling the clock
	GPIO_PeriClockControl(GPIOD, ENABLE);

	//initialize
	GPIO_Init(&GPIOBtn);



	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);

	return 0;
}


void EXTI9_5_IRQHandler(void)
{
	delay(); //200ms, wait till button de-bouncing is over
	GPIO_IRQHandling(GPIO_PIN_NO_5);	//clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
