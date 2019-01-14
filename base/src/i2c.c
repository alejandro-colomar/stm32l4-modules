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
 *	@date		2018/dec/30
 *	@brief		I2C
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>
	#include <string.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/i2c.h"


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
/* Volatile ------------------------------------------------------------------*/
/* Global --------------------------------------------------------------------*/
	uint8_t			i2c_buff [I2C_BUFF_SIZE];
/* Static --------------------------------------------------------------------*/
static	bool			init_pending	= true;
static	I2C_HandleTypeDef	i2c;


/******************************************************************************
 ******* static functions (declarations) **************************************
 ******************************************************************************/
static	void	i2c_msp_init		(void);
static	void	i2c_msp_deinit		(void);
static	void	i2c_gpio_init		(void);
static	void	i2c_gpio_deinit		(void);
static	void	i2c_nvic_conf		(void);
static	void	i2c_nvic_deconf		(void);

static	int	i2c_peripherial_init	(void);
static	int	i2c_peripherial_deinit	(void);
static	int	i2c_filter_analog_conf	(void);
static	int	i2c_filter_digital_conf	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize I2C
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	i2c_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	i2c_msp_init();

	if (i2c_peripherial_init()) {
		prj_error	|= ERROR_I2C_HAL_I2C_INIT;
		prj_error_handle();
		goto err_peripherial;
	}
	if (i2c_filter_analog_conf()) {
		prj_error	|= ERROR_I2C_HAL_I2C_FILTER_A;
		prj_error_handle();
		goto err_filter;
	}
	if (i2c_filter_digital_conf()) {
		prj_error	|= ERROR_I2C_HAL_I2C_FILTER_D;
		prj_error_handle();
		goto err_filter;
	}


	return	ERROR_OK;


err_filter:
	if (i2c_peripherial_deinit()) {
		prj_error	|= ERROR_I2C_HAL_I2C_DEINIT;
		prj_error_handle();
	}

err_peripherial:
	i2c_msp_deinit();

	return	ERROR_NOK;
}

	/**
	 * @brief	Denitialize I2C
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	i2c_deinit	(void)
{

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	ERROR_OK;
	}

	if (i2c_peripherial_deinit()) {
		prj_error	|= ERROR_I2C_HAL_I2C_DEINIT;
		prj_error_handle();
		return	ERROR_NOK;
	}

	i2c_msp_deinit();

	return	ERROR_OK;
}

	/**
	 * @brief	Check I2C slave
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	i2c_chk_slave	(uint8_t addr)
{

	if (init_pending) {
		if (i2c_init()) {
			prj_error	|= ERROR_I2C_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	if (HAL_I2C_IsDeviceReady(&i2c, addr << 1, I2C_TRIALS, I2C_TIMEOUT)) {
		prj_error	|= ERROR_I2C_NOT_READY;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Transmit the message in data through I2C
	 * @param	addr:		address of slave
	 * @param	data_len:	length of data
	 * @param	data:		data to transmit
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	i2c_msg_write	(uint8_t addr, uint8_t data_len, const uint8_t data [data_len])
{
	uint8_t	buff [data_len];

	if (init_pending) {
		if (i2c_init()) {
			prj_error	|= ERROR_I2C_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	memcpy(buff, data, data_len);

	/* XXX:  << 1 is because of HAL bug */
	if (HAL_I2C_Master_Transmit_IT(&i2c, addr << 1, buff, data_len)) {
		prj_error	|= ERROR_I2C_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Request the slave for data
	 * @param	addr:		address of slave
	 * @param	data_len:	length of data to be received
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	i2c_msg_ask	(uint8_t addr, uint8_t data_len)
{

	if (init_pending) {
		if (i2c_init()) {
			prj_error	|= ERROR_I2C_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	/* XXX:  << 1 is because of HAL bug */
	if (HAL_I2C_Master_Receive_IT(&i2c, addr << 1, i2c_buff, data_len)) {
		prj_error	|= ERROR_I2C_RECEIVE;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Check if i2c is ready
	 * @retval	ready:	true if ready
	 */
bool	i2c_msg_ready	(void)
{

	return	(HAL_I2C_STATE_READY == HAL_I2C_GetState(&i2c));
}

	/**
	 * @brief	Read the data received
	 * @param	data_len:	length of data
	 * @param	data:		array where data is to be written
	 * @return	Error
	 * @note	i2c_msg_ask() should have been called before
	 * @note	Sets global variable 'prj_error'
	 */
int	i2c_msg_read	(uint8_t data_len, uint8_t data [data_len])
{
	int	i;

	if (init_pending) {
		if (i2c_init()) {
			prj_error	|= ERROR_I2C_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	if (!i2c_msg_ready()) {
		prj_error	|= ERROR_I2C_NOT_READY;
		return	ERROR_NOK;
	}

	for (i = 0; i < data_len; i++) {
		data[i]	= i2c_buff[i];
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	Handle I2C event interrupt request
	 */
void	I2C1_EV_IRQHandler		(void)
{

	HAL_I2C_EV_IRQHandler(&i2c);
}

	/**
	 * @brief	Handle I2C error interrupt request.
	 */
void	I2C1_ER_IRQHandler		(void)
{

	HAL_I2C_ER_IRQHandler(&i2c);
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	i2c_msp_init		(void)
{

	__HAL_RCC_I2C1_CLK_ENABLE();
	i2c_gpio_init();
	i2c_nvic_conf();
}

static	void	i2c_msp_deinit		(void)
{

	i2c_nvic_deconf();
	i2c_gpio_deinit();
	__HAL_RCC_I2C1_CLK_DISABLE();
}

static	void	i2c_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio.Pin	= GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Mode	= GPIO_MODE_AF_OD;
	gpio.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Pull	= GPIO_NOPULL;
	gpio.Alternate	= GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &gpio);
}

static	void	i2c_gpio_deinit		(void)
{

	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
}

static	void	i2c_nvic_conf		(void)
{

	HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C_PRIORITY, I2C_SUBPRIORITY);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

static	void	i2c_nvic_deconf		(void)
{

	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
}

static	int	i2c_peripherial_init	(void)
{

	i2c.Instance		= I2C1;
	i2c.Init.Timing			= I2C_TIMING;
	i2c.Init.OwnAddress1		= I2C_ADDRESS;
	i2c.Init.AddressingMode		= I2C_ADDRESSINGMODE_7BIT;
	i2c.Init.DualAddressMode	= I2C_DUALADDRESS_DISABLE;
	i2c.Init.OwnAddress2		= I2C_ADDRESS;
	i2c.Init.GeneralCallMode	= I2C_GENERALCALL_DISABLE;
	i2c.Init.NoStretchMode		= I2C_NOSTRETCH_DISABLE;

	return	HAL_I2C_Init(&i2c);
}

static	int	i2c_peripherial_deinit	(void)
{

	return	HAL_I2C_DeInit(&i2c);
}

static	int	i2c_filter_analog_conf	(void)
{

	return	HAL_I2CEx_ConfigAnalogFilter(&i2c, I2C_ANALOGFILTER_ENABLE);
}

static	int	i2c_filter_digital_conf	(void)
{

	return	HAL_I2CEx_ConfigDigitalFilter(&i2c, 0);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
