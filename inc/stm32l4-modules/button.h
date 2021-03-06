/******************************************************************************
 *	Copyright (C) 2019	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2019	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		button.h
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	LGPL-2.0-only
 *	@date		2019/jan/14
 *	@brief		Button
 */


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# ifndef		STM32L4_MODULES_BUTTON_H
	# define	STM32L4_MODULES_BUTTON_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/


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
extern	volatile	bool	button_interrupt;


/******************************************************************************
 ******* functions ************************************************************
 ******************************************************************************/
void	button_init	(void);
void	button_deinit	(void);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* button.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
