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
# define	SPIx_INSTANCE			(SPI1)
# define	SPIx_CLK_ENABLE()		__HAL_RCC_SPI1_CLK_ENABLE()
# define	SPIx_CLK_DISABLE()		__HAL_RCC_SPI1_CLK_DISABLE()

# define	SPIx_nSS_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
# define	SPIx_nSS_GPIO_PORT		(GPIOA)
# define	SPIx_nSS_GPIO_PIN		(GPIO_PIN_4)
# define	SPIx_nSS_GPIO_MODE		(GPIO_MODE_AF_PP)
# define	SPIx_nSS_GPIO_SPEED		(GPIO_SPEED_FREQ_HIGH)
# define	SPIx_nSS_GPIO_PULL		(GPIO_NOPULL)
# define	SPIx_nSS_GPIO_ALT		(GPIO_AF5_SPI1)

# define	SPIx_SCK_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
# define	SPIx_SCK_GPIO_PORT		(GPIOA)
# define	SPIx_SCK_GPIO_PIN		(GPIO_PIN_5)
# define	SPIx_SCK_GPIO_MODE		(GPIO_MODE_AF_PP)
# define	SPIx_SCK_GPIO_SPEED		(GPIO_SPEED_FREQ_HIGH)
# define	SPIx_SCK_GPIO_PULL		(GPIO_NOPULL)
# define	SPIx_SCK_GPIO_ALT		(GPIO_AF5_SPI1)

# define	SPIx_MISO_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
# define	SPIx_MISO_GPIO_PORT		(GPIOA)
# define	SPIx_MISO_GPIO_PIN		(GPIO_PIN_6)
# define	SPIx_MISO_GPIO_MODE		(GPIO_MODE_AF_PP)
# define	SPIx_MISO_GPIO_SPEED		(GPIO_SPEED_FREQ_HIGH)
# define	SPIx_MISO_GPIO_PULL		(GPIO_NOPULL)
# define	SPIx_MISO_GPIO_ALT		(GPIO_AF5_SPI1)

# define	SPIx_MOSI_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
# define	SPIx_MOSI_GPIO_PORT		(GPIOA)
# define	SPIx_MOSI_GPIO_PIN		(GPIO_PIN_7)
# define	SPIx_MOSI_GPIO_MODE		(GPIO_MODE_AF_PP)
# define	SPIx_MOSI_GPIO_SPEED		(GPIO_SPEED_FREQ_HIGH)
# define	SPIx_MOSI_GPIO_PULL		(GPIO_NOPULL)
# define	SPIx_MOSI_GPIO_ALT		(GPIO_AF5_SPI1)

# define	SPIx_IRQHandler			SPI1_IRQHandler
# define	SPIx_IRQn			(SPI1_IRQn)
# define	SPIx_PREEMPT_PRIORITY		(1)
# define	SPIx_SUB_PRIORITY		(0)

# define	SPIx_INIT_BAUD_RATE_PRESCALER	(SPI_BAUDRATEPRESCALER_32)
# define	SPIx_INIT_DIRECTION		(SPI_DIRECTION_2LINES)
# define	SPIx_INIT_CLK_PHASE		(SPI_PHASE_1EDGE)
# define	SPIx_INIT_CLK_POLARITY		(SPI_POLARITY_LOW)
# define	SPIx_INIT_DATA_SIZE		(SPI_DATASIZE_8BIT)
# define	SPIx_INIT_FIRST_BIT		(SPI_FIRSTBIT_MSB)
# define	SPIx_INIT_TI_MODE		(SPI_TIMODE_DISABLE)
# define	SPIx_INIT_CRC_CALCULATION	(SPI_CRCCALCULATION_DISABLE)
# define	SPIx_INIT_CRC_POLYNOMIAL	(7)
# define	SPIx_INIT_CRC_LENGTH		(SPI_CRC_LENGTH_8BIT)
# define	SPIx_INIT_NSS			(SPI_NSS_SOFT)
# define	SPIx_INIT_NSSP_MODE		(SPI_NSS_PULSE_DISABLE)
# define	SPIx_INIT_MODE			(SPI_MODE_MASTER)


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
static	void	spi_nvic_conf		(void);
static	void	spi_nvic_deconf		(void);
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
	init_pending	= true;

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
int	spi_msg_write	(uint8_t data_len, uint8_t data [data_len])
{
	uint8_t	buff [data_len];
	int	i;

	if (init_pending) {
		if (spi_init()) {
			prj_error	|= ERROR_SPI_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	for (i = 0; i < data_len; i++) {
		buff[i]	= data[i];
	}

	HAL_GPIO_WritePin(SPIx_nSS_GPIO_PORT, SPIx_nSS_GPIO_PIN, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit_IT(&spi, buff, data_len)) {
		prj_error	|= ERROR_SPI_HAL_SPI_TRANSMIT;
		prj_error_handle();
		return	ERROR_NOK;
	}
	while (HAL_SPI_GetState(&spi) != HAL_SPI_STATE_READY) {
		__NOP();
	}

	HAL_GPIO_WritePin(SPIx_nSS_GPIO_PORT, SPIx_nSS_GPIO_PIN, GPIO_PIN_SET);

	return	ERROR_OK;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	Handle I2C event interrupt request
	 */
void	SPIx_IRQHandler			(void)
{

	HAL_SPI_IRQHandler(&spi);
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	spi_msp_init		(void)
{

	SPIx_CLK_ENABLE();
	spi_gpio_init();
	spi_nvic_conf();
}

static	void	spi_msp_deinit		(void)
{

	spi_nvic_deconf();
	spi_gpio_deinit();
	SPIx_CLK_DISABLE();
}

static	void	spi_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	SPIx_nSS_GPIO_CLK_ENABLE();
	gpio.Pin	= SPIx_nSS_GPIO_PIN;
	gpio.Mode	= SPIx_nSS_GPIO_MODE;
	gpio.Speed	= SPIx_nSS_GPIO_SPEED;
	gpio.Pull	= SPIx_nSS_GPIO_PULL;
	gpio.Alternate	= SPIx_nSS_GPIO_ALT;
	HAL_GPIO_Init(SPIx_nSS_GPIO_PORT, &gpio);

	SPIx_SCK_GPIO_CLK_ENABLE();
	gpio.Pin	= SPIx_SCK_GPIO_PIN;
	gpio.Mode	= SPIx_SCK_GPIO_MODE;
	gpio.Speed	= SPIx_SCK_GPIO_SPEED;
	gpio.Pull	= SPIx_SCK_GPIO_PULL;
	gpio.Alternate	= SPIx_SCK_GPIO_ALT;
	HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &gpio);

	SPIx_MISO_GPIO_CLK_ENABLE();
	gpio.Pin	= SPIx_MISO_GPIO_PIN;
	gpio.Mode	= SPIx_MISO_GPIO_MODE;
	gpio.Speed	= SPIx_MISO_GPIO_SPEED;
	gpio.Pull	= SPIx_MISO_GPIO_PULL;
	gpio.Alternate	= SPIx_MISO_GPIO_ALT;
	HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &gpio);

	SPIx_MOSI_GPIO_CLK_ENABLE();
	gpio.Pin	= SPIx_MOSI_GPIO_PIN;
	gpio.Mode	= SPIx_MOSI_GPIO_MODE;
	gpio.Speed	= SPIx_MOSI_GPIO_SPEED;
	gpio.Pull	= SPIx_MOSI_GPIO_PULL;
	gpio.Alternate	= SPIx_MOSI_GPIO_ALT;
	HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &gpio);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

static	void	spi_gpio_deinit		(void)
{

	HAL_GPIO_DeInit(SPIx_nSS_GPIO_PORT, SPIx_nSS_GPIO_PIN);
	HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_GPIO_PIN);
	HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_GPIO_PIN);
	HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_GPIO_PIN);
}

static	void	spi_nvic_conf		(void)
{

	HAL_NVIC_SetPriority(SPIx_IRQn, SPIx_PREEMPT_PRIORITY, SPIx_SUB_PRIORITY);
	HAL_NVIC_EnableIRQ(SPIx_IRQn);
}

static	void	spi_nvic_deconf		(void)
{

	HAL_NVIC_DisableIRQ(SPIx_IRQn);
}

static	int	spi_peripherial_init	(void)
{

	spi.Instance		= SPIx_INSTANCE;
	spi.Init.BaudRatePrescaler	= SPIx_INIT_BAUD_RATE_PRESCALER;
	spi.Init.Direction		= SPIx_INIT_DIRECTION;
	spi.Init.CLKPhase		= SPIx_INIT_CLK_PHASE;
	spi.Init.CLKPolarity		= SPIx_INIT_CLK_POLARITY;
	spi.Init.DataSize		= SPIx_INIT_DATA_SIZE;
	spi.Init.FirstBit		= SPIx_INIT_FIRST_BIT;
	spi.Init.TIMode			= SPIx_INIT_TI_MODE;
	spi.Init.CRCCalculation		= SPIx_INIT_CRC_CALCULATION;
	spi.Init.CRCPolynomial		= SPIx_INIT_CRC_POLYNOMIAL;
	spi.Init.CRCLength		= SPIx_INIT_CRC_LENGTH;
	spi.Init.NSS			= SPIx_INIT_NSS;
	spi.Init.NSSPMode		= SPIx_INIT_NSSP_MODE;
	spi.Init.Mode			= SPIx_INIT_MODE;

	return	HAL_SPI_Init(&spi);
}

static	int	spi_peripherial_deinit	(void)
{

	return	HAL_SPI_DeInit(&spi);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
