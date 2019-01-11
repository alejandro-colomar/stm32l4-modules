/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		display.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/16
 *	@brief		display
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
/* Standard C ----------------------------------------------------------------*/
	#include <stdbool.h>
	#include <stdint.h>
		/* memcpy() */
	#include <string.h>

/* Drivers -------------------------------------------------------------------*/
	#include "stm32l4xx_hal.h"

/* libalx --------------------------------------------------------------------*/
/* STM32L4 modules -----------------------------------------------------------*/
	#include "errors.h"
	#include "spi.h"

	#include "display.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	DISPLAY_CODE_DISABLE_MAX7219	(0x0C00u)
	# define	DISPLAY_CODE_DISABLE_TEST_MODE	(0x0F00u)
	# define	DISPLAY_CODE_ENABLE_8_DIGITS	(0x0BFFu)
	# define	DISPLAY_CODE_SET_MAX_BRIGHTNESS	(0x0A0Fu)
	# define	DISPLAY_CODE_DISABLE_BCD_MODE	(0x0900u)
	# define	DISPLAY_CODE_ENABLE_MAX7219	(0x0C01u)

	# define	DISPLAY_DATA_CHAR_0	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x38u | DISPLAY_ROW(0),		\
					0x44u | DISPLAY_ROW(1),		\
					0x44u | DISPLAY_ROW(2),		\
					0x44u | DISPLAY_ROW(3),		\
					0x44u | DISPLAY_ROW(4),		\
					0x44u | DISPLAY_ROW(5),		\
					0x44u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_1	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x10u | DISPLAY_ROW(0),		\
					0x30u | DISPLAY_ROW(1),		\
					0x10u | DISPLAY_ROW(2),		\
					0x10u | DISPLAY_ROW(3),		\
					0x10u | DISPLAY_ROW(4),		\
					0x10u | DISPLAY_ROW(5),		\
					0x10u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_2	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x38u | DISPLAY_ROW(0),		\
					0x44u | DISPLAY_ROW(1),		\
					0x04u | DISPLAY_ROW(2),		\
					0x04u | DISPLAY_ROW(3),		\
					0x08u | DISPLAY_ROW(4),		\
					0x10u | DISPLAY_ROW(5),		\
					0x20u | DISPLAY_ROW(6),		\
					0x7Cu | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_3	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x38u | DISPLAY_ROW(0),		\
					0x44u | DISPLAY_ROW(1),		\
					0x04u | DISPLAY_ROW(2),		\
					0x18u | DISPLAY_ROW(3),		\
					0x04u | DISPLAY_ROW(4),		\
					0x04u | DISPLAY_ROW(5),		\
					0x44u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_4	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x04u | DISPLAY_ROW(0),		\
					0x0Cu | DISPLAY_ROW(1),		\
					0x14u | DISPLAY_ROW(2),		\
					0x24u | DISPLAY_ROW(3),		\
					0x44u | DISPLAY_ROW(4),		\
					0x7Cu | DISPLAY_ROW(5),		\
					0x04u | DISPLAY_ROW(6),		\
					0x04u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_5	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x7Cu | DISPLAY_ROW(0),		\
					0x40u | DISPLAY_ROW(1),		\
					0x40u | DISPLAY_ROW(2),		\
					0x78u | DISPLAY_ROW(3),		\
					0x04u | DISPLAY_ROW(4),		\
					0x04u | DISPLAY_ROW(5),		\
					0x44u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_6	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x38u | DISPLAY_ROW(0),		\
					0x44u | DISPLAY_ROW(1),		\
					0x40u | DISPLAY_ROW(2),		\
					0x78u | DISPLAY_ROW(3),		\
					0x44u | DISPLAY_ROW(4),		\
					0x44u | DISPLAY_ROW(5),		\
					0x44u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_7	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x7Cu | DISPLAY_ROW(0),		\
					0x04u | DISPLAY_ROW(1),		\
					0x04u | DISPLAY_ROW(2),		\
					0x08u | DISPLAY_ROW(3),		\
					0x10u | DISPLAY_ROW(4),		\
					0x20u | DISPLAY_ROW(5),		\
					0x20u | DISPLAY_ROW(6),		\
					0x20u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_8	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x38u | DISPLAY_ROW(0),		\
					0x44u | DISPLAY_ROW(1),		\
					0x44u | DISPLAY_ROW(2),		\
					0x38u | DISPLAY_ROW(3),		\
					0x44u | DISPLAY_ROW(4),		\
					0x44u | DISPLAY_ROW(5),		\
					0x44u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_9	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x38u | DISPLAY_ROW(0),		\
					0x44u | DISPLAY_ROW(1),		\
					0x44u | DISPLAY_ROW(2),		\
					0x44u | DISPLAY_ROW(3),		\
					0x3Cu | DISPLAY_ROW(4),		\
					0x04u | DISPLAY_ROW(5),		\
					0x44u | DISPLAY_ROW(6),		\
					0x38u | DISPLAY_ROW(7),		\
				})
	# define	DISPLAY_DATA_CHAR_BLANK	\
				((uint16_t [DISPLAY_ROWS]) {		\
					0x00u | DISPLAY_ROW(0),		\
					0x00u | DISPLAY_ROW(1),		\
					0x00u | DISPLAY_ROW(2),		\
					0x00u | DISPLAY_ROW(3),		\
					0x00u | DISPLAY_ROW(4),		\
					0x00u | DISPLAY_ROW(5),		\
					0x00u | DISPLAY_ROW(6),		\
					0x00u | DISPLAY_ROW(7),		\
				})


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


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
