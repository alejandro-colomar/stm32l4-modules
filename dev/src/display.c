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
	#include <stdbool.h>
	#include <stdint.h>
	#include <string.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"
	#include "stm32l4-modules/spi.h"

	#include "stm32l4-modules/dev/display.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
#define DISPLAY_CODE_DISABLE_MAX7219		((const uint8_t [2]){	\
							0xCu, 0x00u	\
						})
#define DISPLAY_CODE_DISABLE_TEST_MODE		((const uint8_t [2]){	\
							0x0F, 0x00u	\
						})
#define DISPLAY_CODE_ENABLE_8_DIGITS		((const uint8_t [2]){	\
							0x0B, 0xFFu	\
						})
#define DISPLAY_CODE_SET_MAX_BRIGHTNESS		((const uint8_t [2]){	\
							0x0A, 0x0Fu	\
						})
#define DISPLAY_CODE_DISABLE_BCD_MODE		((const uint8_t [2]){	\
							0x09, 0x00u	\
						})
#define DISPLAY_CODE_ENABLE_MAX7219		((const uint8_t [2]){	\
							0x0C, 0x01u	\
						})

#define DISPLAY_DATA_CHAR_0						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x38u},	\
					{DISPLAY_ROW(1), 0x44u},	\
					{DISPLAY_ROW(2), 0x44u},	\
					{DISPLAY_ROW(3), 0x44u},	\
					{DISPLAY_ROW(4), 0x44u},	\
					{DISPLAY_ROW(5), 0x44u},	\
					{DISPLAY_ROW(6), 0x44u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_1						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x10u},	\
					{DISPLAY_ROW(1), 0x30u},	\
					{DISPLAY_ROW(2), 0x10u},	\
					{DISPLAY_ROW(3), 0x10u},	\
					{DISPLAY_ROW(4), 0x10u},	\
					{DISPLAY_ROW(5), 0x10u},	\
					{DISPLAY_ROW(6), 0x10u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_2						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x38u},	\
					{DISPLAY_ROW(1), 0x44u},	\
					{DISPLAY_ROW(2), 0x04u},	\
					{DISPLAY_ROW(3), 0x04u},	\
					{DISPLAY_ROW(4), 0x08u},	\
					{DISPLAY_ROW(5), 0x10u},	\
					{DISPLAY_ROW(6), 0x20u},	\
					{DISPLAY_ROW(7), 0x7Cu}		\
				})
#define DISPLAY_DATA_CHAR_3						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x38u},	\
					{DISPLAY_ROW(1), 0x44u},	\
					{DISPLAY_ROW(2), 0x04u},	\
					{DISPLAY_ROW(3), 0x18u},	\
					{DISPLAY_ROW(4), 0x04u},	\
					{DISPLAY_ROW(5), 0x04u},	\
					{DISPLAY_ROW(6), 0x44u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_4						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x04u},	\
					{DISPLAY_ROW(1), 0x0Cu},	\
					{DISPLAY_ROW(2), 0x14u},	\
					{DISPLAY_ROW(3), 0x24u},	\
					{DISPLAY_ROW(4), 0x44u},	\
					{DISPLAY_ROW(5), 0x7Cu},	\
					{DISPLAY_ROW(6), 0x04u},	\
					{DISPLAY_ROW(7), 0x04u}		\
				})
#define DISPLAY_DATA_CHAR_5						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x7Cu},	\
					{DISPLAY_ROW(1), 0x40u},	\
					{DISPLAY_ROW(2), 0x40u},	\
					{DISPLAY_ROW(3), 0x78u},	\
					{DISPLAY_ROW(4), 0x04u},	\
					{DISPLAY_ROW(5), 0x04u},	\
					{DISPLAY_ROW(6), 0x44u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_6						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x38u},	\
					{DISPLAY_ROW(1), 0x44u},	\
					{DISPLAY_ROW(2), 0x40u},	\
					{DISPLAY_ROW(3), 0x78u},	\
					{DISPLAY_ROW(4), 0x44u},	\
					{DISPLAY_ROW(5), 0x44u},	\
					{DISPLAY_ROW(6), 0x44u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_7						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x7Cu},	\
					{DISPLAY_ROW(1), 0x04u},	\
					{DISPLAY_ROW(2), 0x04u},	\
					{DISPLAY_ROW(3), 0x08u},	\
					{DISPLAY_ROW(4), 0x10u},	\
					{DISPLAY_ROW(5), 0x20u},	\
					{DISPLAY_ROW(6), 0x20u},	\
					{DISPLAY_ROW(7), 0x20u}		\
				})
#define DISPLAY_DATA_CHAR_8						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x38u},	\
					{DISPLAY_ROW(1), 0x44u},	\
					{DISPLAY_ROW(2), 0x44u},	\
					{DISPLAY_ROW(3), 0x38u},	\
					{DISPLAY_ROW(4), 0x44u},	\
					{DISPLAY_ROW(5), 0x44u},	\
					{DISPLAY_ROW(6), 0x44u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_9						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x38u},	\
					{DISPLAY_ROW(1), 0x44u},	\
					{DISPLAY_ROW(2), 0x44u},	\
					{DISPLAY_ROW(3), 0x44u},	\
					{DISPLAY_ROW(4), 0x3Cu},	\
					{DISPLAY_ROW(5), 0x04u},	\
					{DISPLAY_ROW(6), 0x44u},	\
					{DISPLAY_ROW(7), 0x38u}		\
				})
#define DISPLAY_DATA_CHAR_BLANK						\
				((const uint8_t [DISPLAY_ROWS] [2]){	\
					{DISPLAY_ROW(0), 0x00u},	\
					{DISPLAY_ROW(1), 0x00u},	\
					{DISPLAY_ROW(2), 0x00u},	\
					{DISPLAY_ROW(3), 0x00u},	\
					{DISPLAY_ROW(4), 0x00u},	\
					{DISPLAY_ROW(5), 0x00u},	\
					{DISPLAY_ROW(6), 0x00u},	\
					{DISPLAY_ROW(7), 0x00u}		\
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
static	bool	init_pending	= true;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	display_start		(void);
static	int	display_data_set	(char c, uint8_t data [DISPLAY_ROWS] [2]);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Init LED matrix display using SPI
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	display_init	(void)
{

	if (!init_pending)
		return	ERROR_OK;

	if (spi_init()) {
		prj_error	|= ERROR_DISPLAY_SPI_INIT;
		prj_error_handle();
		goto err_init;
	}
	if (display_start()) {
		prj_error	|= ERROR_DISPLAY_START;
		prj_error_handle();
		goto err_display;
	}

	init_pending	= false;

	return	ERROR_OK;


err_display:
	if (spi_deinit()) {
		prj_error	|= ERROR_DISPLAY_SPI_DEINIT;
		prj_error_handle();
	}
err_init:

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinit LED matrix display using SPI
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	display_deinit	(void)
{
	int	status;

	if (!init_pending)
		return	ERROR_OK;

	init_pending	= true;

	status	= ERROR_OK;

	if (spi_deinit()) {
		prj_error	|= ERROR_DISPLAY_SPI_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}

	return	status;
}

	/**
	 * @brief	Set the display manually
	 * @param	data:	Data to be sent to display
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	display_set	(uint8_t data [DISPLAY_ROWS] [2])
{
	int		i;

	if (init_pending) {
		if (display_init()) {
			prj_error	|= ERROR_DISPLAY_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	for (i = 0; i < DISPLAY_ROWS; i++) {
		if (spi_msg_write(2, data[i])) {
			prj_error	|= ERROR_DISPLAY_SPI_MSG_WRITE;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Show ch on the display
	 * @param	ch:	Character to be displayed
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	display_set_ch	(char c)
{
	uint8_t	data [DISPLAY_ROWS] [2];

	if (init_pending) {
		if (display_init()) {
			prj_error	|= ERROR_DISPLAY_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	if (display_data_set(c, data)) {
		prj_error	|= ERROR_DISPLAY_CHAR;
		prj_error_handle();
		return	ERROR_NOK;
	}

	display_set(data);

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	display_start		(void)
{

	if (spi_msg_write(2, DISPLAY_CODE_DISABLE_MAX7219))
		return	ERROR_NOK;
	if (spi_msg_write(2, DISPLAY_CODE_DISABLE_TEST_MODE))
		return	ERROR_NOK;
	if (spi_msg_write(2, DISPLAY_CODE_ENABLE_8_DIGITS))
		return	ERROR_NOK;
	if (spi_msg_write(2, DISPLAY_CODE_SET_MAX_BRIGHTNESS))
		return	ERROR_NOK;
	if (spi_msg_write(2, DISPLAY_CODE_DISABLE_BCD_MODE))
		return	ERROR_NOK;
	if (spi_msg_write(2, DISPLAY_CODE_ENABLE_MAX7219))
		return	ERROR_NOK;

	return	ERROR_OK;
}

static	int	display_data_set	(char c, uint8_t data [DISPLAY_ROWS] [2])
{

	switch (c) {
	case '0':
		memcpy(data, DISPLAY_DATA_CHAR_0, sizeof(DISPLAY_DATA_CHAR_0));
		break;
	case '1':
		memcpy(data, DISPLAY_DATA_CHAR_1, sizeof(DISPLAY_DATA_CHAR_1));
		break;
	case '2':
		memcpy(data, DISPLAY_DATA_CHAR_2, sizeof(DISPLAY_DATA_CHAR_2));
		break;
	case '3':
		memcpy(data, DISPLAY_DATA_CHAR_3, sizeof(DISPLAY_DATA_CHAR_3));
		break;
	case '4':
		memcpy(data, DISPLAY_DATA_CHAR_4, sizeof(DISPLAY_DATA_CHAR_4));
		break;
	case '5':
		memcpy(data, DISPLAY_DATA_CHAR_5, sizeof(DISPLAY_DATA_CHAR_5));
		break;
	case '6':
		memcpy(data, DISPLAY_DATA_CHAR_6, sizeof(DISPLAY_DATA_CHAR_6));
		break;
	case '7':
		memcpy(data, DISPLAY_DATA_CHAR_7, sizeof(DISPLAY_DATA_CHAR_7));
		break;
	case '8':
		memcpy(data, DISPLAY_DATA_CHAR_8, sizeof(DISPLAY_DATA_CHAR_8));
		break;
	case '9':
		memcpy(data, DISPLAY_DATA_CHAR_9, sizeof(DISPLAY_DATA_CHAR_9));
		break;
	case ' ':
		memcpy(data, DISPLAY_DATA_CHAR_BLANK, sizeof(DISPLAY_DATA_CHAR_BLANK));
		break;
	default:
		memcpy(data, DISPLAY_DATA_CHAR_BLANK, sizeof(DISPLAY_DATA_CHAR_BLANK));
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
