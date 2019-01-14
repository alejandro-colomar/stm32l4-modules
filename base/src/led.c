/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		led.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/04
 *	@brief		LED
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/led.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
# define	LED_G_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
# define	LED_G_GPIO_PORT			(GPIOA)
# define	LED_G_GPIO_PIN			(GPIO_PIN_5)
# define	LED_G_GPIO_MODE			(GPIO_MODE_OUTPUT_PP)
# define	LED_G_GPIO_SPEED		(GPIO_SPEED_FREQ_LOW)
# define	LED_G_GPIO_PULL			(GPIO_NOPULL)


/******************************************************************************
 ******* enums ****************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* structs **************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* variables ************************************************************
 ******************************************************************************/
/* Volatile ------------------------------------------------------------------*/
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/
static	bool	init_pending	= true;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	void	led_gpio_init	(void);
static	void	led_gpio_deinit	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Init LED in GPIO_PIN_5
	 */
void	led_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return;
	}

	led_gpio_init();

	led_reset();
}

	/**
	 * @brief	Deinit LED in GPIO_PIN_5
	 */
void	led_deinit	(void)
{

	if (!init_pending) {
		init_pending	= true;
	} else {
		return;
	}

	led_reset();

	led_gpio_deinit();
}

	/**
	 * @brief	LED on
	 */
void	led_set		(void)
{

	if (init_pending) {
		led_init();
	}

	HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_GPIO_PIN, GPIO_PIN_SET);
}

	/**
	 * @brief	LED off
	 */
void	led_reset	(void)
{

	if (init_pending) {
		led_init();
	}

	HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_GPIO_PIN, GPIO_PIN_RESET);
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	led_gpio_init	(void)
{
	GPIO_InitTypeDef	gpio;
	
	LED_G_GPIO_CLK_ENABLE();
	gpio.Pin	= LED_G_GPIO_PIN;
	gpio.Mode	= LED_G_GPIO_MODE;
	gpio.Speed	= LED_G_GPIO_SPEED;
	gpio.Pull	= LED_G_GPIO_PULL;
	HAL_GPIO_Init(LED_G_GPIO_PORT, &gpio);
}

static	void	led_gpio_deinit	(void)
{

	HAL_GPIO_DeInit(LED_G_GPIO_PORT, LED_G_GPIO_PIN);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
