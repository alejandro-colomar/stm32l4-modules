/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	GPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		can_test.h
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	GPL-2.0-only
 *	@date		2018/dec/30
 *	@brief		CAN test
 */


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# ifndef		CAN_TEST_H
	# define	CAN_TEST_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
/* Standard C ----------------------------------------------------------------*/
	#include <stdnoreturn.h>

/* Drivers -------------------------------------------------------------------*/
/* libalx --------------------------------------------------------------------*/
/* STM32L4 modules -----------------------------------------------------------*/


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


/******************************************************************************
 ******* functions ************************************************************
 ******************************************************************************/
noreturn void	can_w_test	(void);
noreturn void	can_r_test	(void);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* can_test.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
