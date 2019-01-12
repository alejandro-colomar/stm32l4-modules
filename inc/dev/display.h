/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		display.h
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/17
 *	@brief		display
 */


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# ifndef		DISPLAY_H
	# define	DISPLAY_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
/* Standard C ----------------------------------------------------------------*/
	#include <stdint.h>

/* Drivers -------------------------------------------------------------------*/
/* libalx --------------------------------------------------------------------*/
/* STM32L4 modules -----------------------------------------------------------*/


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	DISPLAY_ROWS	(8)
	# define	DISPLAY_COLS	(8)
	# define	DISPLAY_ROW(r)	(((uint16_t)(r) + 1) << DISPLAY_COLS)


/******************************************************************************
 ******* enums ****************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* structs **************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* variables ************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* functions ************************************************************
 ******************************************************************************/
int	display_init	(void);
int	display_deinit	(void);
int	display_set	(uint16_t data [DISPLAY_ROWS]);
int	display_set_ch	(char ch);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* display.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
