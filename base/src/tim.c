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
	#include "stm32l4-modules/led.h"

	#include "stm32l4-modules/tim.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
#define RESOLUTION_1_US		(1000000u)

#define TIMx_INSTANCE		(TIM3)
#define TIMx_CLK_ENABLE()	__HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_CLK_DISABLE()	__HAL_RCC_TIM3_CLK_DISABLE()

#define TIMx_IRQHandler		TIM3_IRQHandler
#define TIMx_IRQn		(TIM3_IRQn)
#define TIMx_PREEMPT_PRIORITY	(2)
#define TIMx_SUB_PRIORITY	(2)


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
	volatile bool	tim_tim3_interrupt;
	volatile bool	tim_tim4_interrupt;
	volatile bool	*const tim_it_timx_interrupt_ptr = &tim_tim3_interrupt;
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/
static	bool			init_pending	= true;
static	TIM_HandleTypeDef	tim;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	void	tim_it_nvic_conf	(void);
static	void	tim_it_nvic_deconf	(void);
static	int	tim_it_tim_init		(uint16_t period_ms);
static	int	tim_it_tim_deinit	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize periodic interrupts using TIMx
	 * @param	period_us:	period of interrupts in us
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	tim_it_init		(uint16_t period_us)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	*tim_it_timx_interrupt_ptr	= false;

	TIMx_CLK_ENABLE();
	tim_it_nvic_conf();
	if (tim_it_tim_init(period_us)) {
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
	if (tim_it_tim_deinit()) {
		prj_error	|= ERROR_TIM_HAL_TIM_DEINIT;
		prj_error_handle();
	}

err_init:
	tim_it_nvic_deconf();
	TIMx_CLK_DISABLE();
	init_pending	= true;

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize periodic interrupts using TIMx
	 *		Sets global variable 'error'
	 * @return	Error
	 */
int	tim_it_deinit		(void)
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
	if (tim_it_tim_deinit()) {
		prj_error	|= ERROR_TIM_HAL_TIM_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	tim_it_nvic_deconf();
	TIMx_CLK_DISABLE();

	return	status;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	Handle TIMx interrupt request
	 */
void	TIMx_IRQHandler			(void)
{

	HAL_TIM_IRQHandler(&tim);
}

	/**
	 * @brief	TIM callback
	 * @param	tim_ptr:	pointer to a TIM_HandleTypeDef structure
	 */
void	HAL_TIM_PeriodElapsedCallback	(TIM_HandleTypeDef *tim_ptr)
{

	if (tim_ptr->Instance == TIM3) {
		tim_tim3_interrupt	= true;
	} else if (tim_ptr->Instance == TIM4) {
		tim_tim4_interrupt	= true;
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	tim_it_nvic_conf	(void)
{

	HAL_NVIC_SetPriority(TIMx_IRQn, TIMx_PREEMPT_PRIORITY,
					TIMx_SUB_PRIORITY);
	HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

static	void	tim_it_nvic_deconf	(void)
{

	HAL_NVIC_DisableIRQ(TIMx_IRQn);
}

static	int	tim_it_tim_init	(uint16_t period_us)
{

	tim	= (TIM_HandleTypeDef){
		.Instance	= TIMx_INSTANCE,
		.Init		= {
			.Prescaler		= ((SystemCoreClock /
							RESOLUTION_1_US) - 1u),
			.CounterMode		= TIM_COUNTERMODE_UP,
			.Period			= (period_us - 1u),
			.ClockDivision		= TIM_CLOCKDIVISION_DIV1,
			.RepetitionCounter	= 0
		}
	};

	return	HAL_TIM_Base_Init(&tim);
}

static	int	tim_it_tim_deinit	(void)
{

	return	HAL_TIM_Base_DeInit(&tim);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
