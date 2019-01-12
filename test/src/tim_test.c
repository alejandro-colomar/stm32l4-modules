/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	GPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		tim_test.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	GPL-2.0-only
 *	@date		2018/dec/25
 *	@brief		timer test
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
/* Standard C ----------------------------------------------------------------*/
	#include <stdbool.h>
	#include <stdint.h>
/* Drivers -------------------------------------------------------------------*/
	#include "stm32l4xx_hal.h"
/* libalx --------------------------------------------------------------------*/
	#include "libalx/alx_mask.h"
/* STM32L4 modules -----------------------------------------------------------*/
	#include "delay.h"
	#include "errors.h"
	#include "led.h"
	#include "tim.h"

	#include "tim_test.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* enums ****************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* structs **************************************************************
 ******************************************************************************/
struct	Tim_Test_Data {
	float	period_us;
};
typedef	struct Tim_Test_Data	Tim_Test_Data_s;


/******************************************************************************
 ******* variables ************************************************************
 ******************************************************************************/
/* Volatile ------------------------------------------------------------------*/
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	flash		(void *data);
static	int	execution_loop	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Test TIM periodic interrupts
	 * @param	period_us:	period of the interrupts
	 * @return	Error
	 */
int	tim_test	(void)
{
	Tim_Test_Data_s	tim_test_data;

	tim_test_data.period_us	= 65000;

	if (delay_us_init()) {
		return	ERROR_NOK;
	}
	led_init();
	if (tim_tim3_init(tim_test_data.period_us)) {
		return	ERROR_NOK;
	}

	if (tim_callback_push(&flash, (void *)&tim_test_data)) {
		prj_error_handle();
		return	ERROR_NOK;
	}
	if (execution_loop()) {
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	flash		(void *data)
{
	Tim_Test_Data_s	*data_cast;

	data_cast	= (Tim_Test_Data_s *)data;

	led_set();
	delay_us(data_cast->period_us * 0.3);

	led_reset();
	delay_us(data_cast->period_us * 0.3);


	return	ERROR_OK;
}

static	int	execution_loop	(void)
{
	while (true) {
		__WFI();
		if (tim_tim3_interrupt) {
			if (tim_callback_exe()) {
				return	ERROR_NOK;
			}
			tim_tim3_interrupt	= false;
		}
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
