/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		nunchuk.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/30
 *	@brief		Nunchuk
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/delay.h"
	#include "stm32l4-modules/led.h"
	#include "stm32l4-modules/errors.h"
	#include "stm32l4-modules/i2c.h"

	#include "stm32l4-modules/dev/nunchuk.h"



/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	NUNCHUK_ADDRESS			(0x52u)

	# define	NUNCHUK_COMMAND_START_0_LEN	(2)
	# define	NUNCHUK_COMMAND_START_0_DATA	(uint8_t [2]){0x40u, 0x00u}

	# define	NUNCHUK_COMMAND_START_1_LEN	(2)
	# define	NUNCHUK_COMMAND_START_1_DATA	(uint8_t [2]){0xF0u, 0x55u}

	# define	NUNCHUK_COMMAND_START_2_LEN	(2)
	# define	NUNCHUK_COMMAND_START_2_DATA	(uint8_t [2]){0xFBu, 0x00u}

	# define	NUNCHUK_COMMAND_GETDATA_LEN	(1)
	# define	NUNCHUK_COMMAND_GETDATA_DATA	(uint8_t [1]){0x00}

	# define	NUNCHUK_DATA_LEN		(6)
	# define	NUNCHUK_DATA_KEY		(UINT8_C(0x17))


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
static	bool	init_pending	= true;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	nunchuk_start		(void);
static	void	nunchuk_extract_data	(uint8_t buff [NUNCHUK_DATA_LEN],
						Nunchuk_Data_s  *data);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Init nunchuk
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	nunchuk_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	if (i2c_init()) {
		prj_error	|= ERROR_NUNCHUK_I2C_INIT;
		prj_error_handle();
		goto err_init;
	}
	if (i2c_chk_slave(NUNCHUK_ADDRESS)) {
		prj_error	|= ERROR_NUNCHUK_I2C_SLAVE;
		prj_error_handle();
		goto err_slave;
	}
	if (nunchuk_start()) {
		prj_error	|= ERROR_NUNCHUK_START;
		prj_error_handle();
		goto err_nunchuk;
	}

	return	ERROR_OK;


err_nunchuk:
err_slave:
	if (i2c_deinit()) {
		prj_error	|= ERROR_NUNCHUK_I2C_DEINIT;
		prj_error_handle();
	}
err_init:
	init_pending	= true;

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinit nunchuk
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	nunchuk_deinit	(void)
{
	int	status;

	status	= ERROR_OK;

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	status;
	}

	if (i2c_deinit()) {
		prj_error	|= ERROR_NUNCHUK_I2C_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}

	return	status;
}

	/**
	 * @brief	Read nunchuk data
	 * @param	data:	struct that holds all nunchuk data
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	nunchuk_read	(Nunchuk_Data_s *data)
{
	uint8_t		buff [NUNCHUK_DATA_LEN];

	if (init_pending) {
		if (nunchuk_init()) {
			prj_error	|= ERROR_NUNCHUK_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	if (i2c_msg_write(NUNCHUK_ADDRESS, NUNCHUK_COMMAND_GETDATA_LEN,
						NUNCHUK_COMMAND_GETDATA_DATA)) {
		prj_error	|= ERROR_NUNCHUK_I2C_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (!i2c_ready()) {
//		__WFI();
		__NOP();
	}

	if (i2c_msg_read(NUNCHUK_ADDRESS, NUNCHUK_DATA_LEN, buff)) {
		prj_error	|= ERROR_NUNCHUK_I2C_ASK;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (!i2c_ready()) {
//		__WFI();
		__NOP();
	}

	nunchuk_extract_data(buff, data);

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	nunchuk_start		(void)
{

	if (i2c_msg_write(NUNCHUK_ADDRESS, NUNCHUK_COMMAND_START_0_LEN,
						NUNCHUK_COMMAND_START_0_DATA)) {
		prj_error	|= ERROR_NUNCHUK_I2C_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (!i2c_ready()) {
//		__WFI();
		__NOP();
	}

	if (i2c_msg_write(NUNCHUK_ADDRESS, NUNCHUK_COMMAND_START_1_LEN,
						NUNCHUK_COMMAND_START_1_DATA)) {
		prj_error	|= ERROR_NUNCHUK_I2C_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (!i2c_ready()) {
//		__WFI();
		__NOP();
	}

	if (i2c_msg_write(NUNCHUK_ADDRESS, NUNCHUK_COMMAND_START_2_LEN,
						NUNCHUK_COMMAND_START_2_DATA)) {
		prj_error	|= ERROR_NUNCHUK_I2C_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (!i2c_ready()) {
//		__WFI();
		__NOP();
	}

	return	ERROR_OK;
}

static	void	nunchuk_extract_data	(uint8_t buff [NUNCHUK_DATA_LEN],
						Nunchuk_Data_s  *data)
{

	data->jst.x	= buff[0];
	data->jst.y	= buff[1];
	data->acc.x8	= buff[2];
	data->acc.y8	= buff[3];
	data->acc.z8	= buff[4];
	data->acc.x10	= (buff[2] << 2) | (buff[5] & 0x0Cu);
	data->acc.y10	= (buff[3] << 2) | (buff[5] & 0x30u);
	data->acc.z10	= (buff[4] << 2) | (buff[5] & 0xC0u);
	data->btn_c	= !(buff[5] & 0x02);
	data->btn_z	= !(buff[5] & 0x01);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
