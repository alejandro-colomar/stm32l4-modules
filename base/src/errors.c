/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		errors.c
 *	@author		Colomar Andrés, Alejandro
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/13
 *	@brief		errors
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
	#include "alx_mask.h"

/* STM32L4 modules -----------------------------------------------------------*/
		/* delay_us_init(), delay_us() */
	#include "delay.h"
		/* led_init(), led_set(), led_stop() */
	#include "led.h"

	#include "errors.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	ERROR_BIT_LEN			(UINT32_C(2000000))
	# define	ERROR_LONG_PULSE_LEN_US		((uint32_t)(ERROR_BIT_LEN * 0.8))
	# define	ERROR_SHORT_PULSE_LEN_US	((uint32_t)(ERROR_BIT_LEN * 0.2))
	# define	ERROR_LOOP_FOREVER		(false)


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
	uint32_t	prj_error;
/* Static --------------------------------------------------------------------*/


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	void	flash_error	(void);
static	void	flash_long	(void);
static	void	flash_short	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Handle error
	 *		Displays the error value by flashing a led from MSB
	 *		to LSB.  A long flash is a 1 and a short flash is a 0.
	 *		After displaying the value, it resets 'prj_error'.
	 */
void	prj_error_handle	(void)
{
	int	i;

	led_reset();
	delay_us(1000000u);

	for (i = 0; i < 10; i++) {
		led_set();
		delay_us(50000u);
		led_reset();
		delay_us(50000u);
	}

	led_reset();
	delay_us(1000000u);

	flash_error();

	prj_error	= 0;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	flash_error	(void)
{
	int	i;
	bool	bit;

	for (i = 31; i >= 0; i--) {
		bit	= (bool)(prj_error & alx_maskgen_u32(i));

		if (bit) {
			flash_long();
		} else {
			flash_short();
		}
	}
}

static	void	flash_long	(void)
{
	led_set();
	delay_us(ERROR_LONG_PULSE_LEN_US);

	led_reset();
	delay_us(ERROR_BIT_LEN - ERROR_LONG_PULSE_LEN_US);
}

static	void	flash_short	(void)
{
	led_set();
	delay_us(ERROR_SHORT_PULSE_LEN_US);

	led_reset();
	delay_us(ERROR_BIT_LEN - ERROR_SHORT_PULSE_LEN_US);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
