/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		tim.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/25
 *	@brief		timer
 */

/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/tim.h"


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
	volatile	bool	tim_tim3_interrupt;
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/
static	bool			init_pending	= true;
static	TIM_HandleTypeDef	tim;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	tim_tim3_tim_init	(uint16_t period_ms);
static	int	tim_tim3_tim_deinit	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize periodic interrupts using TIM3
	 * @param	freq_hz:	frequency of interrupts in Hz
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	tim_tim3_init		(uint16_t period_us)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	tim_tim3_interrupt	= false;

	__HAL_RCC_TIM3_CLK_ENABLE();
	if (tim_tim3_tim_init(period_us)) {
		prj_error	|= ERROR_TIM_HAL_TIM_INIT;
		prj_error_handle();
		goto err_init;
	}
	if (HAL_TIM_Base_Start_IT(&tim)) {
		prj_error	|= ERROR_TIM_HAL_TIM_START_IT;
		prj_error_handle();
		goto err_start;
	}

	return	ERROR_OK;


err_start:
	if (tim_tim3_tim_deinit()) {
		prj_error	|= ERROR_TIM_HAL_TIM_DEINIT;
		prj_error_handle();
	}

err_init:
	__HAL_RCC_TIM3_CLK_DISABLE();

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize periodic interrupts using TIM3
	 *		Sets global variable 'error'
	 * @return	Error
	 */
int	tim_tim3_deinit		(void)
{
	int	status;

	status	= ERROR_OK;

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	status;
	}

	if (HAL_TIM_Base_Stop_IT(&tim)) {
		prj_error	|= ERROR_TIM_HAL_TIM_STOP_IT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	if (tim_tim3_tim_deinit()) {
		prj_error	|= ERROR_TIM_HAL_TIM_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	__HAL_RCC_TIM3_CLK_DISABLE();

	return	status;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	Handle TIM3 interrupt request
	 */
void	TIM3_IRQHandler			(void)
{

	HAL_TIM_IRQHandler(&tim);
}

	/**
	 * @brief	TIM callback
	 * @param	tim_ptr:	pointer to a TIM_HandleTypeDef structure
	 */
void	HAL_TIM_PeriodElapsedCallback	(TIM_HandleTypeDef *tim_ptr)
{

	if (tim_ptr->Instance == TIM1) {
	} else if (tim_ptr->Instance == TIM2) {
	} else if (tim_ptr->Instance == TIM3) {
		tim_tim3_interrupt	= true;
	} else {
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	tim_tim3_tim_init	(uint16_t period_us)
{

	tim.Instance		= TIM3;
	tim.Init.Prescaler		= (SystemCoreClock / RESOLUTION_1_US) -
									1;
	tim.Init.CounterMode		= TIM_COUNTERMODE_UP;
	tim.Init.Period			= period_us - 1;
	tim.Init.ClockDivision		= 0;
	tim.Init.RepetitionCounter	= 0;

	return	HAL_TIM_Base_Init(&tim);
}

static	int	tim_tim3_tim_deinit	(void)
{

	return	HAL_TIM_Base_DeInit(&tim);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
