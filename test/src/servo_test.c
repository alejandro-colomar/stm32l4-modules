/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	GPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		servo_test.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	GPL-2.0-only
 *	@date		2018/dec/05
 *	@brief		servos test
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/delay.h"
	#include "stm32l4-modules/errors.h"
	#include "stm32l4-modules/led.h"
	#include "stm32l4-modules/dev/servo.h"

	#include "stm32l4-modules/test/servo_test.h"


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
/* Volatile ------------------------------------------------------------------*/
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	servo_test_position	(float pos);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Test servos
	 * @return	Error
	 */
int	servo_test_0	(void)
{

	if (servo_init()) {
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Test servos
	 * @return	Error
	 */
int	servo_test_1	(void)
{
	int	i;

	if (delay_ms_init()) {
		return	ERROR_OK;
	}

	if (servo_init()) {
		return	ERROR_NOK;
	}

	for (i = 00; i <= 90; i+=10) {
		led_set();
		delay_ms(500u);
		led_reset();
		delay_ms(500u);

		if (servo_test_position(-i)) {
			return	ERROR_NOK;
		}

		if (servo_test_position(0)) {
			return	ERROR_NOK;
		}

		if (servo_test_position(i)) {
			return	ERROR_NOK;
		}

		if (servo_test_position(0)) {
			return	ERROR_NOK;
		}
	}

	if (delay_ms_deinit()) {
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Test servos
	 * @return	Error
	 */
int	servo_test_2	(void)
{
	int	i;

	if (delay_ms_init()) {
		return	ERROR_OK;
	}

	if (servo_init()) {
		return	ERROR_NOK;
	}

	for (i = -90; i <= 90; i+=10) {
		led_set();
		delay_ms(100u);
		led_reset();
		delay_ms(100u);

		if (servo_test_position(i)) {
			return	ERROR_NOK;
		}
	}

	for (i = 90; i >= -90; i-=10) {
		led_set();
		delay_ms(200u);
		led_reset();
		delay_ms(200u);

		if (servo_test_position(i)) {
			return	ERROR_NOK;
		}
	}

	if (delay_ms_deinit()) {
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	servo_test_position	(float pos)
{
	if (servo_position_set(SERVO_S1, pos)) {
		return	ERROR_NOK;
	}
	if (servo_position_set(SERVO_S2, pos)) {
		return	ERROR_NOK;
	}
	if (servo_position_set(SERVO_S3, pos)) {
		return	ERROR_NOK;
	}
	if (servo_position_set(SERVO_S4, pos)) {
		return	ERROR_NOK;
	}

	if (delay_ms(1000u)) {
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
