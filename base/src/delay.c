/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		delay.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/nov/27
 *	@brief		delay
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/delay.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	RESOLUTION_1_US		(1000000u)


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
static	bool			init_pending	= true;
static	TIM_HandleTypeDef	tim;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	delay_us_tim_init	(void);
static	int	delay_us_tim_deinit	(void);
static	void	delay_us_delay_init	(uint32_t time_us, uint32_t *overflows);
static	void	delay_us_delay_loop	(uint32_t overflows);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize base time for delay_us()
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	delay_us_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	__HAL_RCC_TIM6_CLK_ENABLE();
	if (delay_us_tim_init()) {
		prj_error	|= ERROR_DELAY_HAL_TIM_INIT;
		prj_error_handle();
		goto err_init;
	}

	return	ERROR_OK;


err_init:
	__HAL_RCC_TIM6_CLK_DISABLE();

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize base time for delay_us()
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	delay_us_deinit	(void)
{
	int	status;

	status	= ERROR_OK;

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	status;
	}

	if (delay_us_tim_deinit()) {
		prj_error	|= ERROR_DELAY_HAL_TIM_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	__HAL_RCC_TIM6_CLK_DISABLE();

	return	status;
}

	/**
	 * @brief	Delay <time_us> microseconds
	 * @param	time_us:	Delay value (us)
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	delay_us	(uint32_t time_us)
{
	uint32_t	overflows;

	if (init_pending) {
		if (delay_us_init()) {
			prj_error	|= ERROR_DELAY_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	if (!time_us) {
		return	ERROR_OK;
	}

	delay_us_delay_init(time_us, &overflows);

	if (HAL_TIM_Base_Start(&tim)) {
		prj_error	|= ERROR_DELAY_HAL_TIM_START;
		prj_error_handle();
		return	ERROR_NOK;
	}

	delay_us_delay_loop(overflows);

	if (HAL_TIM_Base_Stop(&tim)) {
		prj_error	|= ERROR_DELAY_HAL_TIM_STOP;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
	/* Resolution: 1 us */
static	int	delay_us_tim_init	(void)
{

	tim.Instance		= TIM6; 
	tim.Init.Prescaler		= (SystemCoreClock / RESOLUTION_1_US) -
									1u;
	tim.Init.CounterMode		= TIM_COUNTERMODE_UP;
	tim.Init.Period			= UINT16_MAX;
	tim.Init.ClockDivision		= TIM_CLOCKDIVISION_DIV1;
	tim.Init.RepetitionCounter	= 0;
	tim.Init.AutoReloadPreload	= TIM_AUTORELOAD_PRELOAD_DISABLE;

	return	HAL_TIM_Base_Init(&tim);
}

static	int	delay_us_tim_deinit	(void)
{

	return	HAL_TIM_Base_DeInit(&tim);
}

static	void	delay_us_delay_init	(uint32_t time_us, uint32_t *overflows)
{
	uint32_t	counter_initial;
	uint32_t	partial;

	*overflows	= (time_us / ((uint32_t)UINT16_MAX + 1u)) + 1u;
	partial		= time_us % ((uint32_t)UINT16_MAX + 1u);
	counter_initial	= (uint32_t)UINT16_MAX + 1u - partial;

	__HAL_TIM_SET_COUNTER(&tim, counter_initial);
}

static	void	delay_us_delay_loop	(uint32_t overflows)
{
	uint32_t	counter_flags;
	bool		flag;

	counter_flags	= 0;
	while (counter_flags < overflows) {
		flag	= __HAL_TIM_GET_FLAG(&tim, TIM_FLAG_UPDATE);
		if (flag) {
			__HAL_TIM_CLEAR_FLAG(&tim, TIM_FLAG_UPDATE);
			counter_flags++;
		}
	}
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
