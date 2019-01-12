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
/* Standard C ----------------------------------------------------------------*/
	#include <stdbool.h>
	#include <stdint.h>
	#include <stdnoreturn.h>

/* Drivers -------------------------------------------------------------------*/
	#include "stm32l4xx_hal.h"

/* libalx --------------------------------------------------------------------*/
/* STM32L4 modules -----------------------------------------------------------*/
	#include "can.h"
	#include "delay.h"
	#include "errors.h"
	#include "led.h"

	#include "can_test.h"


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
static	void	can_tst_msg_show	(int i, int8_t buff[CAN_DATA_LEN]);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Test can: write
	 */
noreturn void	can_w_test	(void)
{
	int8_t	buff [CAN_DATA_LEN];
	int	i	= 0;

	delay_us_init();
	can_init();

	delay_us(2000000u);
	while (true) {
		i++;
		i	%= 8;

		can_tst_msg_write(i, buff);

		delay_us(2500000u - (2*100000u*i));
	}
}

	/**
	 * @brief	Test can: read
	 */
noreturn void	can_r_test	(void)
{
	int8_t	buff [CAN_DATA_LEN];
	int	i	= 0;

	delay_us_init();
	led_init();
	can_init();

	delay_us(3000000u);
	while (true) {
		i++;
		i	%= 8;

		can_msg_read((uint8_t *)buff);

		can_tst_msg_show(i, buff);

		delay_us(2500000u - (2*100000u*i));
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	can_tst_msg_write	(int i, int8_t buff[CAN_DATA_LEN])
{
	buff[0]	= i;
	buff[1]	= i;
	buff[2]	= i;
	buff[3]	= i;
	buff[4]	= i;
	buff[5]	= i;
	buff[6]	= i;
	buff[7]	= i;

	if (can_msg_write((uint8_t *)buff)) {
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

static	void	can_tst_msg_show	(int i, int8_t buff[CAN_DATA_LEN])
{
	int	j;

	for (j = 0; j < buff[i]; j++) {
		led_set();
		delay_us(100000u);
		led_reset();
		delay_us(100000u);
	}
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/