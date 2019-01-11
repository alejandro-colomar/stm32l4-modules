/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		servo.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/nov/27
 *	@brief		servos
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
	#include "libalx/alx_math.h"
/* STM32L4 modules -----------------------------------------------------------*/
	#include "errors.h"
	#include "pwm.h"

	#include "servo.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
		/* Resolution = 1/1000000 s = 1 us */
	# define	SERVO_PWM_RESOLUTION_s		(1000000u)

		/* Period = 20 ms = 20000 us */
	# define	SERVO_PWM_PERIOD_us		(20000u)

		/* Angle = 0 deg -> Duty time = 1.5 ms -> Duty cycle = 0.075 */
	# define	SERVO_PWM_DUTY_DEF		(0.075f)
		/* Angle = -90 deg -> Duty time = 1 ms -> Duty cycle = 0.05 */
	# define	SERVO_PWM_DUTY_MIN		(0.020f)/*(0.05f) (from datashit)*/
		/* Angle = 90 deg -> Duty time = 2 ms -> Duty cycle = 0.1 */
	# define	SERVO_PWM_DUTY_MAX		(0.120f)/*(0.1f) (from datashit)*/


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
static	float	duty_cycle [SERVO_QTY];
static	bool	init_pending	= true;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	servo_duty_calc	(float position, float *duty);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Init servos
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	servo_init		(void)
{
	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	if (pwm_tim2_init(SERVO_PWM_RESOLUTION_s, SERVO_PWM_PERIOD_us)) {
		prj_error	|= ERROR_SERVO_PWM_INIT;
		prj_error_handle();
		return	ERROR_NOK;
	}

	if (pwm_tim2_chX_set(SERVO_PWM_DUTY_DEF, TIM_CHANNEL_1)) {
		prj_error	|= ERROR_SERVO_PWM_SET;
		prj_error_handle();
		return	ERROR_NOK;
	}
	if (pwm_tim2_chX_set(SERVO_PWM_DUTY_DEF, TIM_CHANNEL_2)) {
		prj_error	|= ERROR_SERVO_PWM_SET;
		prj_error_handle();
		return	ERROR_NOK;
	}
	if (pwm_tim2_chX_set(SERVO_PWM_DUTY_DEF, TIM_CHANNEL_3)) {
		prj_error	|= ERROR_SERVO_PWM_SET;
		prj_error_handle();
		return	ERROR_NOK;
	}
	if (pwm_tim2_chX_set(SERVO_PWM_DUTY_DEF, TIM_CHANNEL_4)) {
		prj_error	|= ERROR_SERVO_PWM_SET;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Set servo sX position
	 * @param	servo:		servo to set
	 * @param	position:	position (deg): [-90, 90]
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	servo_position_set	(int8_t servo, float position)
{
	uint32_t	tim_chan;

	if (init_pending) {
		if (servo_init()) {
			prj_error	|= ERROR_SERVO_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	switch (servo) {
	case SERVO_S1:
		tim_chan	= TIM_CHANNEL_1;
		break;
	case SERVO_S2:
		tim_chan	= TIM_CHANNEL_2;
		break;
	case SERVO_S3:
		tim_chan	= TIM_CHANNEL_3;
		break;
	case SERVO_S4:
		tim_chan	= TIM_CHANNEL_4;
		break;
	default:
		prj_error	|= ERROR_SERVO_ID;
		prj_error_handle();
		return	ERROR_NOK;
	}

	servo_duty_calc(position, &duty_cycle[servo]);
	if (pwm_tim2_chX_set(duty_cycle[servo], tim_chan)) {
		prj_error	|= ERROR_SERVO_PWM_SET;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Stop servos
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	servo_stop		(void)
{
	if (init_pending) {
		return	ERROR_OK;
	}

	if (pwm_tim2_stop()) {
		prj_error	|= ERROR_SERVO_PWM_STOP;
		prj_error_handle();
		return	ERROR_NOK;
	}

	init_pending	= true;

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
	/*
	 * Calculate PWM duty cycle for specified servo position
	 *		position:	position (deg): [-90, 900]
	 *		duty:		Duty cycle: [0, 1]
	 * return:	saturation:
	 *		0 =	OK,
	 *		>0 =	POSITIVE SATURATION,
	 *		<0 =	NEGATIVE SATURATION
	 */
static	int	servo_duty_calc	(float position, float *duty)
{
	int	saturation	= SERVO_SATURATION_OK;

	if (position < (SERVO_ANGLE_MIN)) {
		saturation	= SERVO_SATURATION_NEG;
	}
	if (position > (SERVO_ANGLE_MAX)) {
		saturation	= SERVO_SATURATION_POS;
	}

	switch (saturation) {
	case SERVO_SATURATION_NEG:
		*duty	= SERVO_PWM_DUTY_MIN;
		break;
	case SERVO_SATURATION_POS:
		*duty	= SERVO_PWM_DUTY_MAX;
		break;
	default:
		*duty	= alx_scale_linear_f(position,
				SERVO_ANGLE_MIN, SERVO_ANGLE_MAX,
				SERVO_PWM_DUTY_MIN, SERVO_PWM_DUTY_MAX);
		break;
	}

	return	saturation;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
