/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	GPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		can_test.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	GPL-2.0-only
 *	@date		2018/dec/30
 *	@brief		CAN test
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>
	#include <string.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/can.h"
	#include "stm32l4-modules/delay.h"
	#include "stm32l4-modules/errors.h"
	#include "stm32l4-modules/led.h"

	#include "stm32l4-modules/test/can_test.h"


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
static	int	can_tst_msg_write	(int i, int8_t buff[CAN_DATA_LEN]);
static	int	can_tst_msg_show	(int i, int8_t buff[CAN_DATA_LEN]);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Test can: write
	 */
int	can_w_test	(void)
{
	int8_t	buff [CAN_DATA_LEN];
	int	i	= 0;

	if (delay_us_init())
		return	ERROR_NOK;
	if (can_init())
		return	ERROR_NOK;

	if (delay_us(2000000u))
		return	ERROR_NOK;

	while (true) {
		i++;
		i	%= 8;

		if (can_tst_msg_write(i, buff))
			return	ERROR_NOK;

		if (delay_us(2500000u - (2*100000u*i)))
			return	ERROR_NOK;
	}
}

	/**
	 * @brief	Test can: read
	 */
int	can_r_test	(void)
{
	int8_t	buff [CAN_DATA_LEN];
	int	i	= 0;

	if (delay_us_init())
		return	ERROR_NOK;
	led_init();
	if (can_init())
		return	ERROR_NOK;

	if (delay_us(3000000u))
		return	ERROR_NOK;

	while (true) {
		i++;
		i	%= 8;

		if (can_msg_read((uint8_t *)buff))
			return	ERROR_NOK;

		if (can_tst_msg_show(i, buff))
			return	ERROR_NOK;

		if (delay_us(2500000u - (2*100000u*i)))
			return	ERROR_NOK;
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	can_tst_msg_write	(int i, int8_t buff[CAN_DATA_LEN])
{
	memset(buff, i, CAN_DATA_LEN);

	if (can_msg_write((uint8_t *)buff))
		return	ERROR_NOK;

	return	ERROR_OK;
}

static	int	can_tst_msg_show	(int i, int8_t buff[CAN_DATA_LEN])
{
	int	j;

	for (j = 0; j < buff[i]; j++) {
		led_set();
		if (delay_us(100000u))
			return	ERROR_NOK;
		led_reset();
		if (delay_us(100000u))
			return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
