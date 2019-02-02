/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	GPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		nunchuk_test.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	GPL-2.0-only
 *	@date		2018/dec/22
 *	@brief		Nunchuk test
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/delay.h"
	#include "stm32l4-modules/errors.h"
	#include "stm32l4-modules/led.h"
	#include "stm32l4-modules/dev/display.h"
	#include "stm32l4-modules/dev/nunchuk.h"
	#include "stm32l4-modules/dev/servo.h"

	#include "stm32l4-modules/test/nunchuk_test.h"


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
static	void	display_data_clear	(uint16_t display_data[DISPLAY_ROWS]);
static	void	display_data_set	(Nunchuk_Data_s	nunchuk_data,
					uint16_t display_data[DISPLAY_ROWS]);
static	int	nunchuk_servo_set	(Nunchuk_Data_s	nunchuk_data);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Test nunchuk INITIALIZATION
	 * @return	Error
	 */
int	nunchuk_test_0	(void)
{
	led_init();
	if (delay_us_init())
		return	ERROR_NOK;
	if (servo_init())
		return	ERROR_NOK;
	if (nunchuk_init())
		return	ERROR_NOK;

	led_set();

	return	ERROR_OK;
}

	/**
	 * @brief	Test nunchuk
	 * @return	Error
	 */
int	nunchuk_test_1	(void)
{
	Nunchuk_Data_s	nunchuk_data;
	uint16_t	display_data [DISPLAY_ROWS];

	if (delay_us_init())
		return	ERROR_NOK;
	if (servo_init())
		return	ERROR_NOK;
	if (nunchuk_init())
		return	ERROR_NOK;

	do {
		if (nunchuk_read(&nunchuk_data))
			return	ERROR_NOK;

		display_data_set(nunchuk_data, display_data);

		if (delay_us(1000000u))
			return	ERROR_NOK;
	} while (!nunchuk_data.btn_z);

	return	ERROR_OK;
}

	/**
	 * @brief	Test nunchuk
	 * @return	Error
	 */
int	nunchuk_test_2	(void)
{
	Nunchuk_Data_s	nunchuk_data;

	led_init();
	if (delay_us_init())
		return	ERROR_NOK;
	if (servo_init())
		return	ERROR_NOK;
	if (nunchuk_init())
		return	ERROR_NOK;

	do {
		if (nunchuk_read(&nunchuk_data)) {
			prj_error_handle();
			return	ERROR_NOK;
		}

		led_set();
		if (delay_us(100000u))
			return	ERROR_NOK;

		if (nunchuk_servo_set(nunchuk_data))
			return	ERROR_NOK;

		led_reset();
		if (delay_us(100000u))
			return	ERROR_NOK;
	} while (true);
//	} while (!nunchuk_data.btn_z);

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	display_data_clear	(uint16_t display_data[DISPLAY_ROWS])
{
	int	i;

	for (i = 0; i < DISPLAY_ROWS; i++)
		display_data[i]	= DISPLAY_ROW(i);
}

static	void	display_data_set	(Nunchuk_Data_s	nunchuk_data,
					uint16_t display_data[DISPLAY_ROWS])
{
	display_data_clear(display_data);

	display_data[0]	|= nunchuk_data.jst.x;
	display_data[1]	|= nunchuk_data.jst.y;

	if (!nunchuk_data.btn_c) {
		display_data[2]	|= nunchuk_data.acc.x8;
		display_data[3]	|= nunchuk_data.acc.y8;
		display_data[4]	|= nunchuk_data.acc.z8;
	} else {
		display_data[5]	|= (nunchuk_data.acc.x10 >> 2);
		display_data[6]	|= (nunchuk_data.acc.y10 >> 2);
		display_data[7]	|= (nunchuk_data.acc.z10 >> 2);
	}
}

static	int	nunchuk_servo_set	(Nunchuk_Data_s	nunchuk_data)
{
	float	pos;

	pos	= (float)nunchuk_data.acc.x8 / UINT8_MAX * 90.0;
	if (servo_position_set(SERVO_S1, pos))
		return	ERROR_NOK;

	pos	= (float)nunchuk_data.acc.y8 / UINT8_MAX * 90.0;
	if (servo_position_set(SERVO_S2, pos))
		return	ERROR_NOK;

	pos	= (float)nunchuk_data.acc.z8 / UINT8_MAX * 90.0;
	if (servo_position_set(SERVO_S3, pos))
		return	ERROR_NOK;

	pos	= (float)nunchuk_data.jst.x / UINT8_MAX * 90.0;
	if (servo_position_set(SERVO_S4, pos))
		return	ERROR_NOK;

	return	ERROR_OK;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
