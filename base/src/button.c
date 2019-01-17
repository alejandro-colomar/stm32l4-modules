/******************************************************************************
 *	Copyright (C) 2019	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2019	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		button.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	LGPL-2.0-only
 *	@date		2019/jan/14
 *	@brief		Button
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/button.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
# define	BUTTON_EXTI_LINE		(GPIO_PIN_13)

# define	BUTTON_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOC_CLK_ENABLE()
# define	BUTTON_GPIO_PORT		(GPIOC)
# define	BUTTON_GPIO_PIN			(BUTTON_EXTI_LINE)
# define	BUTTON_GPIO_MODE		(GPIO_MODE_IT_FALLING)
# define	BUTTON_GPIO_PULL		(GPIO_NOPULL)

# define	EXTIx_IRQHandler		EXTI15_10_IRQHandler
# define	EXTIx_IRQn			(EXTI15_10_IRQn)
# define	EXTIx_PREEMPT_PRIORITY		(2)
# define	EXTIx_SUB_PRIORITY		(0)


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
	volatile	bool	button_interrupt;
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/
static	bool	init_pending	= true;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	void	button_gpio_init	(void);
static	void	button_gpio_deinit	(void);
static	void	button_nvic_conf	(void);
static	void	button_nvic_deconf	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Init button in GPIO_PIN_13
	 */
void	button_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return;
	}

	button_gpio_init();
	button_nvic_conf();

	button_interrupt	= false;
}

	/**
	 * @brief	Deinit button in GPIO_PIN_13
	 */
void	button_deinit	(void)
{

	if (!init_pending) {
		init_pending	= true;
	} else {
		return;
	}

	button_nvic_deconf();
	button_gpio_deinit();

	button_interrupt	= false;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	Handle EXTIx interrupt request
	 */
void	EXTIx_IRQHandler	(void)
{

	if (__HAL_GPIO_EXTI_GET_IT(BUTTON_EXTI_LINE)) {
		HAL_GPIO_EXTI_IRQHandler(BUTTON_GPIO_PIN);
	}
}

	/**
	 * @brief	Callback for EXTI
	 * @param	gpio_pin: Pin of interrupt
	 */
void	HAL_GPIO_EXTI_Callback	(uint16_t  gpio_pin)
{

	switch (gpio_pin) {
	case BUTTON_GPIO_PIN:
		button_interrupt	= true;
		break;
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	button_gpio_init	(void)
{
	GPIO_InitTypeDef	gpio	= {
		.Pin	= BUTTON_GPIO_PIN,
		.Mode	= BUTTON_GPIO_MODE,
		.Pull	= BUTTON_GPIO_PULL
	};

	BUTTON_GPIO_CLK_ENABLE();
	HAL_GPIO_Init(BUTTON_GPIO_PORT, &gpio);
}

static	void	button_gpio_deinit	(void)
{

	HAL_GPIO_DeInit(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN);
}

static	void	button_nvic_conf	(void)
{

	HAL_NVIC_SetPriority(EXTIx_IRQn, EXTIx_PREEMPT_PRIORITY,
					EXTIx_SUB_PRIORITY);
	HAL_NVIC_EnableIRQ(EXTIx_IRQn);
}

static	void	button_nvic_deconf	(void)
{

	HAL_NVIC_DisableIRQ(EXTIx_IRQn);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
