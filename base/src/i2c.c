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
#define I2Cx_TRIALS			(10)
#define I2Cx_TIMEOUT			(10000)

#define I2Cx_INSTANCE			(I2C1)
#define I2Cx_CLK_ENABLE()		__HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_CLK_DISABLE()		__HAL_RCC_I2C1_CLK_DISABLE()

#define I2Cx_GPIO_MODE			(GPIO_MODE_AF_OD)
#define I2Cx_GPIO_PULL			(GPIO_NOPULL)
#define I2Cx_GPIO_SPEED			(GPIO_SPEED_FREQ_VERY_HIGH)

#define I2Cx_SCL_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_PORT		(GPIOB)
#define I2Cx_SCL_GPIO_PIN		(GPIO_PIN_6)
#define I2Cx_SCL_GPIO_MODE		(I2Cx_GPIO_MODE)
#define I2Cx_SCL_GPIO_PULL		(I2Cx_GPIO_PULL)
#define I2Cx_SCL_GPIO_SPEED		(I2Cx_GPIO_SPEED)
#define I2Cx_SCL_GPIO_ALT		(GPIO_AF4_I2C1)

#define I2Cx_SDA_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SDA_GPIO_PORT		(GPIOB)
#define I2Cx_SDA_GPIO_PIN		(GPIO_PIN_7)
#define I2Cx_SDA_GPIO_MODE		(I2Cx_GPIO_MODE)
#define I2Cx_SDA_GPIO_PULL		(I2Cx_GPIO_PULL)
#define I2Cx_SDA_GPIO_SPEED		(I2Cx_GPIO_SPEED)
#define I2Cx_SDA_GPIO_ALT		(GPIO_AF4_I2C1)

#define I2Cx_EV_IRQHandler		I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler		I2C1_ER_IRQHandler
#define I2Cx_EV_IRQn			(I2C1_EV_IRQn)
#define I2Cx_PREEMPT_PRIORITY		(1)
#define I2Cx_SUB_PRIORITY		(1)

#define I2Cx_INIT_TIMING		(0xF0330309u)
#define I2Cx_INIT_OWN_ADRESS_1		(0x00u)
#define I2Cx_INIT_ADDRESSING_MODE	(I2C_ADDRESSINGMODE_7BIT)
#define I2Cx_INIT_DUAL_ADDRESS_MODE	(I2C_DUALADDRESS_DISABLE)
#define I2Cx_INIT_OWN_ADRESS_2		(I2Cx_INIT_OWN_ADRESS_1)
#define I2Cx_INIT_GENERAL_CALL_MODE	(I2C_GENERALCALL_DISABLE)
#define I2Cx_INIT_NO_STRETCH_MODE	(I2C_NOSTRETCH_DISABLE)

#define I2Cx_FILTER_A_CONF		(I2C_ANALOGFILTER_ENABLE)
#define I2Cx_FILTER_D_CONF		(0)


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

	if (!init_pending)
		return	ERROR_OK;

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

	init_pending	= false;

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

	if (init_pending)
		return	ERROR_OK;

	init_pending	= true;

	if (i2c_peripherial_deinit()) {
		prj_error	|= ERROR_I2C_HAL_I2C_DEINIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	i2c_msp_deinit();

	return	ERROR_OK;
}

	/**
	 * @brief	Check if i2c is ready
	 * @retval	ready:	true if ready
	 */
bool	i2c_ready	(void)
{

	return	(HAL_I2C_GetState(&i2c) == HAL_I2C_STATE_READY);
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

	if (HAL_I2C_IsDeviceReady(&i2c, addr << 1, I2Cx_TRIALS, I2Cx_TIMEOUT)) {
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
int	i2c_msg_write	(uint8_t addr, uint8_t data_len,
				const uint8_t data [data_len])
{
	uint8_t	buff [data_len];

	if (init_pending) {
		if (i2c_init()) {
			prj_error	|= ERROR_I2C_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	memcpy(buff, data, sizeof(buff));
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
int	i2c_msg_read	(uint8_t addr, uint8_t data_len, uint8_t buff [data_len])
{

	if (init_pending) {
		if (i2c_init()) {
			prj_error	|= ERROR_I2C_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	/* XXX:  << 1 is because of HAL bug */
	if (HAL_I2C_Master_Receive_IT(&i2c, addr << 1, buff, data_len)) {
		prj_error	|= ERROR_I2C_RECEIVE;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	Handle I2C event interrupt request
	 */
void	I2Cx_EV_IRQHandler		(void)
{

	HAL_I2C_EV_IRQHandler(&i2c);
}

	/**
	 * @brief	Handle I2C error interrupt request.
	 */
void	I2Cx_ER_IRQHandler		(void)
{

	HAL_I2C_ER_IRQHandler(&i2c);
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	i2c_msp_init		(void)
{

	I2Cx_CLK_ENABLE();
	i2c_gpio_init();
	i2c_nvic_conf();
}

static	void	i2c_msp_deinit		(void)
{

	i2c_nvic_deconf();
	i2c_gpio_deinit();
	I2Cx_CLK_DISABLE();
}

static	void	i2c_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	I2Cx_SCL_GPIO_CLK_ENABLE();
	gpio	= (GPIO_InitTypeDef){
		.Pin		= I2Cx_SCL_GPIO_PIN,
		.Mode		= I2Cx_SCL_GPIO_MODE,
		.Pull		= I2Cx_SCL_GPIO_PULL,
		.Speed		= I2Cx_SCL_GPIO_SPEED,
		.Alternate	= I2Cx_SCL_GPIO_ALT
	};
	HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &gpio);

	I2Cx_SDA_GPIO_CLK_ENABLE();
	gpio	= (GPIO_InitTypeDef){
		.Pin		= I2Cx_SDA_GPIO_PIN,
		.Mode		= I2Cx_SDA_GPIO_MODE,
		.Pull		= I2Cx_SDA_GPIO_PULL,
		.Speed		= I2Cx_SDA_GPIO_SPEED,
		.Alternate	= I2Cx_SDA_GPIO_ALT
	};
	HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &gpio);
}

static	void	i2c_gpio_deinit		(void)
{

	HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_GPIO_PIN);
	HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_GPIO_PIN);
}

static	void	i2c_nvic_conf		(void)
{

	HAL_NVIC_SetPriority(I2Cx_EV_IRQn, I2Cx_PREEMPT_PRIORITY,
						I2Cx_SUB_PRIORITY);
	HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
}

static	void	i2c_nvic_deconf		(void)
{

	HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);
}

static	int	i2c_peripherial_init	(void)
{

	i2c	= (I2C_HandleTypeDef){
		.Instance		= I2Cx_INSTANCE,
		.Init	= {
			.Timing			= I2Cx_INIT_TIMING,
			.OwnAddress1		= I2Cx_INIT_OWN_ADRESS_1,
			.AddressingMode		= I2Cx_INIT_ADDRESSING_MODE,
			.DualAddressMode	= I2Cx_INIT_DUAL_ADDRESS_MODE,
			.OwnAddress2		= I2Cx_INIT_OWN_ADRESS_2,
			.GeneralCallMode	= I2Cx_INIT_GENERAL_CALL_MODE,
			.NoStretchMode		= I2Cx_INIT_NO_STRETCH_MODE
		}
	};

	return	HAL_I2C_Init(&i2c);
}

static	int	i2c_peripherial_deinit	(void)
{

	return	HAL_I2C_DeInit(&i2c);
}

static	int	i2c_filter_analog_conf	(void)
{

	return	HAL_I2CEx_ConfigAnalogFilter(&i2c, I2Cx_FILTER_A_CONF);
}

static	int	i2c_filter_digital_conf	(void)
{

	return	HAL_I2CEx_ConfigDigitalFilter(&i2c, I2Cx_FILTER_D_CONF);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
