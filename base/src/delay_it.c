/******************************************************************************
 *	Copyright (C) 2019	Colomar Andrés, Alejandro		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		delay_it.c
 *	@author		Colomar Andrés, Alejandro
 *	@copyright	LGPL-2.0-only
 *	@date		2019/jan/17
 *	@brief		delay_it
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

	#include "stm32l4-modules/delay_it.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
#define RESOLUTION_100_US	(10000u)
#define RESOLUTION_1_MS		(1000u)

#define TIMx_INSTANCE		(TIM4)
#define TIMx_CLK_ENABLE()	__HAL_RCC_TIM4_CLK_ENABLE()
#define TIMx_CLK_DISABLE()	__HAL_RCC_TIM4_CLK_DISABLE()

#define TIMx_IRQHandler		TIM4_IRQHandler
#define TIMx_IRQn		(TIM4_IRQn)
#define TIMx_PREEMPT_PRIORITY	(2)
#define TIMx_SUB_PRIORITY	(3)


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
static	volatile bool	*const delay_it_timx_interrupt_ptr = &tim_tim4_interrupt;
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/
static	bool			init_pending	= true;
static	TIM_HandleTypeDef	tim;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	delay_it_tim_init	(void);
static	void	delay_it_delay_init	(uint32_t time_ms, uint32_t *overflows);
static	void	delay_it_delay_loop	(uint32_t overflows);
static	int	delay_it_tim_deinit	(void);
static	void	delay_it_nvic_conf	(void);
static	void	delay_it_nvic_deconf	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize base time for delay_it_ms()
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	delay_it_ms_init	(void)
{

	if (!init_pending)
		return	ERROR_OK;

	TIMx_CLK_ENABLE();
	delay_it_nvic_conf();
	if (delay_it_tim_init()) {
		prj_error	|= ERROR_DELAY_HAL_TIM_INIT;
		prj_error_handle();
		goto err_init;
	}

	init_pending	= false;

	return	ERROR_OK;


err_init:
	delay_it_nvic_deconf();
	TIMx_CLK_DISABLE();

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize base time for delay_it_ms()
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	delay_it_ms_deinit	(void)
{
	int	status;

	if (init_pending)
		return	ERROR_OK;

	init_pending	= true;

	status	= ERROR_OK;

	if (delay_it_tim_deinit()) {
		prj_error	|= ERROR_DELAY_HAL_TIM_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	delay_it_nvic_deconf();
	TIMx_CLK_DISABLE();

	return	status;
}

	/**
	 * @brief	Delay <time_ms> microseconds
	 * @param	time_ms:	Delay value (ms)
	 * 			This value should not exceed (UINT32_MAX / 10)
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	delay_it_ms		(uint32_t time_ms)
{
	uint32_t	overflows;

	if (!time_ms)
		return	ERROR_OK;

	if (init_pending) {
		if (delay_it_ms_init()) {
			prj_error	|= ERROR_DELAY_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	delay_it_delay_init(time_ms, &overflows);

	if (HAL_TIM_Base_Start(&tim)) {
		prj_error	|= ERROR_DELAY_HAL_TIM_START;
		prj_error_handle();
		return	ERROR_NOK;
	}

	delay_it_delay_loop(overflows);

	if (HAL_TIM_Base_Stop(&tim)) {
		prj_error	|= ERROR_DELAY_HAL_TIM_STOP;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
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


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	delay_it_tim_init	(void)
{

	tim	= (TIM_HandleTypeDef){
		.Instance	= TIMx_INSTANCE,
		.Init		= {
			.Prescaler		= ((SystemCoreClock /
							RESOLUTION_100_US) - 1u),
			.CounterMode		= TIM_COUNTERMODE_UP,
			.Period			= UINT16_MAX,
			.ClockDivision		= TIM_CLOCKDIVISION_DIV1,
			.RepetitionCounter	= 0,
			.AutoReloadPreload	= TIM_AUTORELOAD_PRELOAD_DISABLE
		}
	};

	return	HAL_TIM_Base_Init(&tim);
}

static	void	delay_it_delay_init	(uint32_t time_ms, uint32_t *overflows)
{
	uint32_t	time_res;
	uint32_t	counter_initial;
	uint32_t	partial;

	time_res	= time_ms * (RESOLUTION_100_US / RESOLUTION_1_MS);

	*overflows	= (time_res / ((uint32_t)UINT16_MAX + 1u)) + 1u;
	partial		= time_res % ((uint32_t)UINT16_MAX + 1u);
	counter_initial	= (uint32_t)UINT16_MAX + 1u - partial;

	__HAL_TIM_SET_COUNTER(&tim, counter_initial);
}

static	void	delay_it_delay_loop	(uint32_t overflows)
{
	uint32_t	counter_flags	= 0;

	while (counter_flags < overflows) {
		__WFI();

		if (*delay_it_timx_interrupt_ptr) {
			counter_flags++;

			*delay_it_timx_interrupt_ptr	= false;
		}
	}
}

static	int	delay_it_tim_deinit	(void)
{

	return	HAL_TIM_Base_DeInit(&tim);
}

static	void	delay_it_nvic_conf		(void)
{

	HAL_NVIC_SetPriority(TIMx_IRQn, TIMx_PREEMPT_PRIORITY,
					TIMx_SUB_PRIORITY);
	HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

static	void	delay_it_nvic_deconf		(void)
{

	HAL_NVIC_DisableIRQ(TIMx_IRQn);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
