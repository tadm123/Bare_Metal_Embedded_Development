/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: May 22, 2024
 *      Author: danielramirez
 */


#include "stm32f407xx.h"


#define ENABLE 			1
#define BTN_PRESSED 	ENABLE
#define MY_ADDR			0x61
#define SLAVE_ADDR		0x68

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

I2C_Handle_t I2C1Handle;

// some data
uint8_t some_data[] = "We are testing I2C master Tx\n";		//send less than 32 bytes in one transaction, Arduino supports max 32 bytes in one transaction


/*
 * PB6-> SCL
 * PB9-> SCA
 *
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = 	GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = 	GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 	4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = 	GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2CConfig.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2CConfig.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2CConfig.I2C_FMDutyCycle = I2C_FM_DUTY_2; 	//wont use this, can put any
	I2C1Handle.I2CConfig.I2C_SLCSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}


void GPIO_ButtonInit(void)
{

	GPIO_Handle_t GPIOBtn, GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig,GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);
}


int main (void)
{
	//GPIO Button Init
	GPIO_ButtonInit();

	//I2C pin Inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//enable the I2C peripheral control
	I2C_PeripheralControl(I2C1, ENABLE);

	//wait for button press
	while(1)
	{
		//wait till button is pressed
		while (! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing delay 200ms
		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);
	}

	return 0;
}


