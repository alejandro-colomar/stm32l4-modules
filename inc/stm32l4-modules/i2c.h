/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		i2c.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/18
 *	@brief		I2C
 */


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# ifndef		STM32L4_MODULES_I2C_H
	# define	STM32L4_MODULES_I2C_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	I2C_TIMING	(0xF0330309u)
	# define	I2C_ADDRESS	(0x00u)
	# define	I2C_BUFF_SIZE	(UINT8_MAX)
	# define	I2C_PRIORITY	(1)
	# define	I2C_SUBPRIORITY	(1)
	# define	I2C_TRIALS	(10)
	# define	I2C_TIMEOUT	(10000)


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
int	i2c_init	(void);
int	i2c_deinit	(void);
int	i2c_chk_slave	(uint8_t addr);
int	i2c_msg_write	(uint8_t addr, uint8_t data_len, const uint8_t data [data_len]);
int	i2c_msg_ask	(uint8_t addr, uint8_t data_len);
bool	i2c_msg_ready	(void);
int	i2c_msg_read	(uint8_t data_len, uint8_t data [data_len]);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* i2c.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
