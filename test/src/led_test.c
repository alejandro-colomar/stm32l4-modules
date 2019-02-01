/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	GPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		led_test.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	GPL-2.0-only
 *	@date		2018/dec/13
 *	@brief		LED test
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/delay.h"
	#include "stm32l4-modules/delay_it.h"
	#include "stm32l4-modules/errors.h"
	#include "stm32l4-modules/led.h"

	#include "stm32l4-modules/test/led_test.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* enums ****************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* structs **************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* variables ************************************************************
 ******************************************************************************/
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Test LEDs
	 * @return	Error
	 */
int	led_test_1	(void)
{
	int		i;

	if (delay_us_init())
		return	ERROR_NOK;
	led_init();

	for (i = 0; i <= 100; i++) {
		led_set();
		if (delay_us(1000u * i))
			return	ERROR_NOK;

		led_reset();
		if (delay_us(1000u * i))
			return	ERROR_NOK;
	}

	if (delay_us_deinit())
		return	ERROR_NOK;

	return	ERROR_OK;
}

	/**
	 * @brief	Test LEDs
	 * @return	Error
	 */
int	led_test_2	(void)
{
	int		i;

	if (delay_ms_init())
		return	ERROR_NOK;
	led_init();

	for (i = 0; i <= 100; i++) {
		led_set();
		if (delay_ms(i))
			return	ERROR_NOK;

		led_reset();
		if (delay_ms(i))
			return	ERROR_NOK;
	}

	if (delay_ms_deinit())
		return	ERROR_NOK;

	return	ERROR_OK;
}

	/**
	 * @brief	Test LEDs
	 * @return	Error
	 */
int	led_test_3	(void)
{
	int		i;

	if (delay_it_ms_init())
		return	ERROR_NOK;
	led_init();

	for (i = 0; i <= 100; i++) {
		led_set();
		if (delay_it_ms(i))
			return	ERROR_NOK;

		led_reset();
		if (delay_it_ms(i))
			return	ERROR_NOK;
	}

	if (delay_it_ms_deinit())
		return	ERROR_NOK;

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
