/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		can.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/11
 *	@brief		CAN
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/can.h"


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
/* Volatile ------------------------------------------------------------------*/
static	volatile	bool	can_msg_pending;

/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/
static	bool			init_pending	= true;
static	CAN_HandleTypeDef	can;
static	CAN_TxHeaderTypeDef	can_tx_header;
static	CAN_RxHeaderTypeDef	can_rx_header;
static	uint8_t			can_rx_data [CAN_DATA_LEN];
static	uint32_t		can_tx_mailbox;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	void	can_msp_init		(void);
static	void	can_msp_deinit		(void);
static	void	can_gpio_init		(void);
static	void	can_gpio_deinit		(void);
static	void	can_nvic_conf		(void);
static	void	can_nvic_deconf		(void);

static	int	can_peripherial_init	(void);
static	int	can_peripherial_deinit	(void);
static	int	can_filter_conf		(void);
static	void	can_tx_header_conf	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize CAN
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	can_init	(void)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	can_msg_pending	= false;

	can_msp_init();

	if (can_peripherial_init()) {
		prj_error	|= ERROR_CAN_HAL_CAN_INIT;
		prj_error_handle();
		goto err_peripherial;
	}
	if (can_filter_conf()) {
		prj_error	|= ERROR_CAN_HAL_CAN_FILTER;
		prj_error_handle();
		goto err_filter;
	}
	if (HAL_CAN_Start(&can)) {
		prj_error	|= ERROR_CAN_HAL_CAN_START;
		prj_error_handle();
		goto err_start;
	}
	if (HAL_CAN_ActivateNotification(&can, CAN_IT_RX_FIFO0_MSG_PENDING)) {
		prj_error	|= ERROR_CAN_HAL_CAN_ACTI_NOTIF;
		prj_error_handle();
		goto err_notif;
	}

	can_tx_header_conf();

	return	ERROR_OK;


err_notif:
	if (HAL_CAN_Stop(&can)) {
		prj_error	|= ERROR_CAN_HAL_CAN_STOP;
		prj_error_handle();
	}

err_start:
err_filter:
	if (can_peripherial_deinit()) {
		prj_error	|= ERROR_CAN_HAL_CAN_DEINIT;
		prj_error_handle();
	}

err_peripherial:
	can_msp_deinit();

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize CAN
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	can_deinit	(void)
{
	int	status;

	status	= ERROR_OK;

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	status;
	}

	if (HAL_CAN_DeactivateNotification(&can, CAN_IT_RX_FIFO0_MSG_PENDING)) {
		prj_error	|= ERROR_CAN_HAL_CAN_DEACTI_NOTIF;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	if (HAL_CAN_Stop(&can)) {
		prj_error	|= ERROR_CAN_HAL_CAN_STOP;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	if (can_peripherial_deinit()) {
		prj_error	|= ERROR_CAN_HAL_CAN_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	can_msp_deinit();

	can_msg_pending	= false;

	return	status;
}

	/**
	 * @brief	Transmit data through CAN
	 * @param	data:	data to transmit
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	can_msg_write	(uint8_t data [CAN_DATA_LEN])
{
	uint8_t	can_tx_data [CAN_DATA_LEN];
	int	i;

	if (init_pending) {
		if (can_init()) {
			prj_error	|= ERROR_CAN_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	for (i = 0; i < CAN_DATA_LEN; i++) {
		can_tx_data[i]	= data[i];
	}

	if (HAL_CAN_AddTxMessage(&can, &can_tx_header, can_tx_data,
							&can_tx_mailbox)) {
		prj_error	|= ERROR_CAN_HAL_ADD_TX_MSG;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Read the data received
	 * @param	data:	array where data is to be written
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	can_msg_read	(uint8_t data [CAN_DATA_LEN])
{
	int	status;
	int	i;

	status	= ERROR_OK;

	if (init_pending) {
		if (can_init()) {
			prj_error	|= ERROR_CAN_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	if (!can_msg_pending) {
		prj_error	|= ERROR_CAN_NO_MSG;
		status	= ERROR_NOK;
	}

	for (i = 0; i < CAN_DATA_LEN; i++) {
		data[i]	= can_rx_data[i];
	}

	for (i = 0; i < CAN_DATA_LEN; i++) {
		can_rx_data[i]	= 0;
	}
	can_msg_pending	= false;

	return	status;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	This function handles CAN1 RX0 interrupt request
	 */
void	CAN1_RX0_IRQHandler			(void)
{

	HAL_CAN_IRQHandler(&can);
}

	/**
	 * @brief	Rx Fifo 0 message pending callback
	 * @param	can_ptr:	pointer to a CAN_HandleTypeDef
	 *			structure that contains the configuration
	 *			information for the specified CAN
	 */
void	HAL_CAN_RxFifo0MsgPendingCallback	(CAN_HandleTypeDef *can_ptr)
{

	if (HAL_CAN_GetRxMessage(can_ptr, CAN_RX_FIFO0, &can_rx_header,
								can_rx_data)) {
		prj_error	|= ERROR_CAN_HAL_GET_RX_MSG;
		prj_error_handle();
	} else if (can_msg_pending) {
		prj_error	|= ERROR_CAN_MSG_LOST;
	} else {
		can_msg_pending	= true;
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	void	can_msp_init		(void)
{

	__HAL_RCC_CAN1_CLK_ENABLE();
	can_gpio_init();
	can_nvic_conf();
}

static	void	can_msp_deinit		(void)
{

	can_nvic_deconf();
	can_gpio_deinit();
	__HAL_RCC_CAN1_CLK_DISABLE();
}

	/*
	 * PA12 -> TX
	 * PA11 -> RX
	 */
static	void	can_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio.Pin	= GPIO_PIN_12 | GPIO_PIN_11;
	gpio.Mode	= GPIO_MODE_AF_OD;
	gpio.Speed	= GPIO_SPEED_FREQ_LOW;
	gpio.Pull	= GPIO_NOPULL;
	gpio.Alternate	= GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOA, &gpio);
}

	/*
	 * PA12 -> TX
	 * PA11 -> RX
	 */
static	void	can_gpio_deinit		(void)
{

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12 | GPIO_PIN_11);
}

static	void	can_nvic_conf		(void)
{

	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

static	void	can_nvic_deconf		(void)
{

	HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
}

	/*
	 * CAN clock = 1 MHz = 80 MHz / 80;
	 * Period = 1 us
	 */
static	int	can_peripherial_init	(void)
{

	can.Instance		= CAN1;
	can.Init.TimeTriggeredMode	= DISABLE;
	can.Init.AutoBusOff		= DISABLE;
	can.Init.AutoWakeUp		= DISABLE;
	can.Init.AutoRetransmission	= ENABLE;
	can.Init.ReceiveFifoLocked	= DISABLE;
	can.Init.TransmitFifoPriority	= DISABLE;
	can.Init.Mode			= CAN_MODE_NORMAL;
	can.Init.SyncJumpWidth		= CAN_SJW_1TQ;
	can.Init.TimeSeg1		= CAN_BS1_4TQ;
	can.Init.TimeSeg2		= CAN_BS2_5TQ;
	can.Init.Prescaler		= SystemCoreClock / 1000000u;

	return	HAL_CAN_Init(&can);
}

static	int	can_peripherial_deinit	(void)
{

	return	HAL_CAN_DeInit(&can);
}

static	int	can_filter_conf		(void)
{
	CAN_FilterTypeDef	can_filter;

	can_filter.FilterIdHigh		= 0x0000u;
	can_filter.FilterIdLow		= 0x0000u;
	can_filter.FilterMaskIdHigh	= 0x0000u;
	can_filter.FilterMaskIdLow	= 0x0000u;
	can_filter.FilterFIFOAssignment	= CAN_FILTER_FIFO0;
	can_filter.FilterBank		= 5;
	can_filter.FilterMode		= CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale		= CAN_FILTERSCALE_16BIT;
	can_filter.FilterActivation	= ENABLE;

	return	HAL_CAN_ConfigFilter(&can, &can_filter);
}

static	void	can_tx_header_conf	(void)
{

	can_tx_header.StdId			= 0x3AAu;
	can_tx_header.ExtId			= 0x00u;
	can_tx_header.IDE			= CAN_ID_STD;
	can_tx_header.RTR			= CAN_RTR_DATA;
	can_tx_header.DLC			= CAN_DATA_LEN;
	can_tx_header.TransmitGlobalTime	= DISABLE;
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
