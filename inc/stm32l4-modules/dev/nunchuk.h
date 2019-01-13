/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		nunchuk.h
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/22
 *	@brief		Nunchuk
 */


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# ifndef		STM32L4_MODULES_NUNCHUK_H
	# define	STM32L4_MODULES_NUNCHUK_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	NUNCHUK_DATA_LEN		(6)


/******************************************************************************
 ******* enums ****************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* structs **************************************************************
 ******************************************************************************/
struct	Nunchuk_Joystick_Data {
	uint8_t		x;
	uint8_t		y;
};
typedef	struct Nunchuk_Joystick_Data	Nunchuk_Joystick_Data_s;

struct	Nunchuk_Accelerometer_Data {
	uint8_t		x8;
	uint8_t		y8;
	uint8_t		z8;
	uint16_t	x10;
	uint16_t	y10;
	uint16_t	z10;
};
typedef	struct Nunchuk_Accelerometer_Data	Nunchuk_Accelerometer_Data_s;

struct	Nunchuk_Data {
	Nunchuk_Joystick_Data_s		jst;
	Nunchuk_Accelerometer_Data_s	acc;
	bool				btn_c;
	bool				btn_z;
};
typedef	struct Nunchuk_Data	Nunchuk_Data_s;


/******************************************************************************
 ******* variables ************************************************************
 ******************************************************************************/


/******************************************************************************
 ******* functions ************************************************************
 ******************************************************************************/
int	nunchuk_init	(void);
int	nunchuk_deinit	(void);
int	nunchuk_read	(Nunchuk_Data_s *data);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* nunchuk.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
