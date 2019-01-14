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
# ifndef		STM32L4_MODULES_DISPLAY_H
	# define	STM32L4_MODULES_DISPLAY_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdint.h>


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	DISPLAY_ROWS	(8)
	# define	DISPLAY_ROW(r)	(((uint8_t)(r) + 1))


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
int	display_set	(uint8_t data [DISPLAY_ROWS] [2]);
int	display_set_ch	(char c);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* display.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
