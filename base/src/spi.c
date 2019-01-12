/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		spi.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/16
 *	@brief		SPI
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/spi.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
	# define	SPI_TIMEOUT	(500)


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
static	bool			init_pending	= true;
static	SPI_HandleTypeDef	spi;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	void	spi_msp_init		(void);
static	void	spi_msp_deinit		(void);
static	void	spi_gpio_init		(void);
static	void	spi_gpio_deinit		(void);
static	int	spi_peripherial_init	(void);
static	int	spi_peripherial_deinit	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize SPI
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	spi_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	spi_msp_init();
	if (spi_peripherial_init()) {
		prj_error	|= ERROR_SPI_HAL_SPI_INIT;
		prj_error_handle();
		goto err_peripherial;
	}

	return	ERROR_OK;


err_peripherial:
	spi_msp_deinit();

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize SPI
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	spi_deinit	(void)
{
	int	status;

	status	= ERROR_OK;

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	status;
	}

	if (spi_peripherial_deinit()) {
		prj_error	|= ERROR_SPI_HAL_SPI_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	spi_msp_deinit();

	return	status;
}

	/**
	 * @brief	Transmit data through SPI
	 * @param	data:	data to transmit
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	spi_msg_write	(uint16_t data)
{
	uint8_t	spi_data [sizeof(uint16_t) / sizeof(uint8_t)];

	if (init_pending) {
		if (spi_init()) {
			prj_error	|= ERROR_SPI_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	spi_data[0]	= data / (UINT8_MAX + 1u);
	spi_data[1]	= data % (UINT8_MAX + 1u);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&spi, spi_data, 1, SPI_TIMEOUT)) {
		prj_error	|= ERROR_SPI_HAL_SPI_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (HAL_SPI_GetState(&spi) != HAL_SPI_STATE_READY) {
		/* Empty loop */
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	spi_msp_init		(void)
{

	__SPI2_CLK_ENABLE();
	spi_gpio_init();
}

static	void	spi_msp_deinit		(void)
{

	spi_gpio_deinit();
	__SPI2_CLK_DISABLE();
}

static	void	spi_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio.Pin	= GPIO_PIN_4;
	gpio.Mode	= GPIO_MODE_AF_PP;
	gpio.Speed	= GPIO_SPEED_FREQ_HIGH;
	gpio.Pull	= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin	= GPIO_PIN_5;
	gpio.Mode	= GPIO_MODE_AF_PP;
	gpio.Speed	= GPIO_SPEED_FREQ_HIGH;
	gpio.Pull	= GPIO_NOPULL;
	gpio.Alternate	= GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin	= GPIO_PIN_6;
	gpio.Mode	= GPIO_MODE_AF_PP;
	gpio.Speed	= GPIO_SPEED_FREQ_HIGH;
	gpio.Pull	= GPIO_NOPULL;
	gpio.Alternate	= GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin	= GPIO_PIN_7;
	gpio.Mode	= GPIO_MODE_AF_PP;
	gpio.Speed	= GPIO_SPEED_FREQ_HIGH;
	gpio.Pull	= GPIO_NOPULL;
	gpio.Alternate	= GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &gpio);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

static	void	spi_gpio_deinit		(void)
{

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
}

static	int	spi_peripherial_init	(void)
{

	spi.Instance		= SPI1;
	spi.Init.BaudRatePrescaler	= SPI_BAUDRATEPRESCALER_32;
	spi.Init.Direction		= SPI_DIRECTION_2LINES;
	spi.Init.CLKPhase		= SPI_PHASE_1EDGE;
	spi.Init.CLKPolarity		= SPI_POLARITY_LOW;
	spi.Init.CRCCalculation		= SPI_CRCCALCULATION_DISABLE;
	spi.Init.DataSize		= SPI_DATASIZE_16BIT;
	spi.Init.FirstBit		= SPI_FIRSTBIT_MSB;
	spi.Init.NSS			= SPI_NSS_SOFT;
	spi.Init.TIMode			= SPI_TIMODE_DISABLE;
	spi.Init.Mode			= SPI_MODE_MASTER;

	return	HAL_SPI_Init(&spi);
}

static	int	spi_peripherial_deinit	(void)
{

	return	HAL_SPI_DeInit(&spi);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
