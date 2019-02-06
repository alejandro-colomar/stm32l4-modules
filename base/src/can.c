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
	#include <string.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/can.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
#define CANx_INSTANCE				(CAN1)
#define CANx_CLK_ENABLE()			__HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_CLK_DISABLE()			__HAL_RCC_CAN1_CLK_DISABLE()

#define CANx_TX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define CANx_TX_GPIO_PORT			(GPIOA)
#define CANx_TX_GPIO_PIN			(GPIO_PIN_12)
#define CANx_TX_GPIO_MODE			(GPIO_MODE_AF_OD)
#define CANx_TX_GPIO_SPEED			(GPIO_SPEED_FREQ_LOW)
#define CANx_TX_GPIO_PULL			(GPIO_NOPULL)
#define CANx_TX_GPIO_ALT			(GPIO_AF9_CAN1)

#define CANx_RX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define CANx_RX_GPIO_PORT			(GPIOA)
#define CANx_RX_GPIO_PIN			(GPIO_PIN_11)
#define CANx_RX_GPIO_MODE			(GPIO_MODE_AF_OD)
#define CANx_RX_GPIO_SPEED			(GPIO_SPEED_FREQ_LOW)
#define CANx_RX_GPIO_PULL			(GPIO_NOPULL)
#define CANx_RX_GPIO_ALT			(GPIO_AF9_CAN1)

#define CANx_RX0_IRQHandler			CAN1_RX0_IRQHandler
#define CANx_RX0_IRQn				(CAN1_RX0_IRQn)
#define CANx_PREEMPT_PRIORITY			(1)
#define CANx_SUB_PRIORITY			(0)

#define CANx_INIT_TIME_TRIGGERED_MODE		(DISABLE)
#define CANx_INIT_AUTO_BUS_OFF			(DISABLE)
#define CANx_INIT_AUTO_WAKE_UP			(DISABLE)
#define CANx_INIT_AUTO_RETRANSMISSION		(ENABLE)
#define CANx_INIT_RECEIVE_FIFO_LOCKED		(DISABLE)
#define CANx_INIT_TRANSMIT_FIFO_PRIORITY	(DISABLE)
#define CANx_INIT_MODE				(CAN_MODE_NORMAL)
#define CANx_INIT_SYNC_JUMP_WIDTH		(CAN_SJW_1TQ)
#define CANx_INIT_TIME_SEG_1			(CAN_BS1_4TQ)
#define CANx_INIT_TIME_SEG_2			(CAN_BS2_5TQ)
#define CANx_INIT_PRESCALER			(SystemCoreClock / 1000000u)

#define CANx_FILTER_ID_HI			(0)
#define CANx_FILTER_ID_LO			(0)
#define CANx_FILTER_MASK_ID_HI			(0)
#define CANx_FILTER_MASK_ID_LO			(0)
#define CANx_FILTER_FIFO_ASSIGNMENT		(CAN_FILTER_FIFO0)
#define CANx_FILTER_BANK			(5)
#define CANx_FILTER_MODE			(CAN_FILTERMODE_IDMASK)
#define CANx_FILTER_SCALE			(CAN_FILTERSCALE_16BIT)
#define CANx_FILTER_ACTIVATION			(ENABLE)

#define CANx_TX_HDR_STD_ID			(0x3AAu)
#define CANx_TX_HDR_EXT_ID			(0)
#define CANx_TX_HDR_IDE				(CAN_ID_STD)
#define CANx_TX_HDR_RTR				(CAN_RTR_DATA)
#define CANx_TX_HDR_DLC				(CAN_DATA_LEN)
#define CANx_TX_HDR_TRANSMISSION_GLOBAL_TIME	(DISABLE)


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

	if (!init_pending)
		return	ERROR_OK;

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

	init_pending	= false;

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

	if (init_pending)
		return	ERROR_OK;

	init_pending	= true;

	status	= ERROR_OK;

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
int	can_msg_write	(const uint8_t data [CAN_DATA_LEN])
{
	uint8_t	can_tx_data [CAN_DATA_LEN];

	if (init_pending) {
		if (can_init()) {
			prj_error	|= ERROR_CAN_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	memcpy(can_tx_data, data, sizeof(can_tx_data));

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

	if (init_pending) {
		if (can_init()) {
			prj_error	|= ERROR_CAN_INIT;
			prj_error_handle();
			return	ERROR_NOK;
		}
	}

	status	= ERROR_OK;

	if (!can_msg_pending) {
		prj_error	|= ERROR_CAN_NO_MSG;
		status	= ERROR_NOK;
	}

	memcpy(data, can_rx_data, sizeof(can_rx_data));
	memset(can_rx_data, 0, sizeof(can_rx_data));
	can_msg_pending	= false;

	return	status;
}


/******************************************************************************
 ******* HAL weak functions (redefinitions) ***********************************
 ******************************************************************************/
	/**
	 * @brief	This function handles CANx RX0 interrupt request
	 */
void	CANx_RX0_IRQHandler			(void)
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

	CANx_CLK_ENABLE();
	can_gpio_init();
	can_nvic_conf();
}

static	void	can_msp_deinit		(void)
{

	can_nvic_deconf();
	can_gpio_deinit();
	CANx_CLK_DISABLE();
}

	/*
	 * PA12 -> TX
	 * PA11 -> RX
	 */
static	void	can_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	CANx_TX_GPIO_CLK_ENABLE();
	gpio	= (const GPIO_InitTypeDef){
		.Pin		= CANx_TX_GPIO_PIN,
		.Mode		= CANx_TX_GPIO_MODE,
		.Speed		= CANx_TX_GPIO_SPEED,
		.Pull		= CANx_TX_GPIO_PULL,
		.Alternate	= CANx_TX_GPIO_ALT
	};
	HAL_GPIO_Init(CANx_TX_GPIO_PORT, &gpio);

	CANx_RX_GPIO_CLK_ENABLE();
	gpio	= (const GPIO_InitTypeDef){
		.Pin		= CANx_RX_GPIO_PIN,
		.Mode		= CANx_RX_GPIO_MODE,
		.Speed		= CANx_RX_GPIO_SPEED,
		.Pull		= CANx_RX_GPIO_PULL,
		.Alternate	= CANx_RX_GPIO_ALT
	};
	HAL_GPIO_Init(CANx_RX_GPIO_PORT, &gpio);
}

	/*
	 * PA12 -> TX
	 * PA11 -> RX
	 */
static	void	can_gpio_deinit		(void)
{

	HAL_GPIO_DeInit(CANx_TX_GPIO_PORT, CANx_TX_GPIO_PIN);
	HAL_GPIO_DeInit(CANx_RX_GPIO_PORT, CANx_RX_GPIO_PIN);
}

static	void	can_nvic_conf		(void)
{

	HAL_NVIC_SetPriority(CANx_RX0_IRQn, CANx_PREEMPT_PRIORITY,
						CANx_SUB_PRIORITY);
	HAL_NVIC_EnableIRQ(CANx_RX0_IRQn);
}

static	void	can_nvic_deconf		(void)
{

	HAL_NVIC_DisableIRQ(CANx_RX0_IRQn);
}

	/*
	 * CAN clock = 1 MHz = 80 MHz / 80;
	 * Period = 1 us
	 */
static	int	can_peripherial_init	(void)
{

	can	= (const CAN_HandleTypeDef){
		.Instance	= CANx_INSTANCE,
		.Init		= {
			.TimeTriggeredMode	= CANx_INIT_TIME_TRIGGERED_MODE,
			.AutoBusOff		= CANx_INIT_AUTO_BUS_OFF,
			.AutoWakeUp		= CANx_INIT_AUTO_WAKE_UP,
			.AutoRetransmission	= CANx_INIT_AUTO_RETRANSMISSION,
			.ReceiveFifoLocked	= CANx_INIT_RECEIVE_FIFO_LOCKED,
			.TransmitFifoPriority	= CANx_INIT_TRANSMIT_FIFO_PRIORITY,
			.Mode			= CANx_INIT_MODE,
			.SyncJumpWidth		= CANx_INIT_SYNC_JUMP_WIDTH,
			.TimeSeg1		= CANx_INIT_TIME_SEG_1,
			.TimeSeg2		= CANx_INIT_TIME_SEG_2,
			.Prescaler		= CANx_INIT_PRESCALER
		}
	};

	return	HAL_CAN_Init(&can);
}

static	int	can_peripherial_deinit	(void)
{

	return	HAL_CAN_DeInit(&can);
}

static	int	can_filter_conf		(void)
{
	CAN_FilterTypeDef	can_filter	= {
		.FilterIdHigh		= CANx_FILTER_ID_HI,
		.FilterIdLow		= CANx_FILTER_ID_LO,
		.FilterMaskIdHigh	= CANx_FILTER_MASK_ID_HI,
		.FilterMaskIdLow	= CANx_FILTER_MASK_ID_LO,
		.FilterFIFOAssignment	= CANx_FILTER_FIFO_ASSIGNMENT,
		.FilterBank		= CANx_FILTER_BANK,
		.FilterMode		= CANx_FILTER_MODE,
		.FilterScale		= CANx_FILTER_SCALE,
		.FilterActivation	= CANx_FILTER_ACTIVATION
	};

	return	HAL_CAN_ConfigFilter(&can, &can_filter);
}

static	void	can_tx_header_conf	(void)
{

	can_tx_header	= (const CAN_TxHeaderTypeDef){
		.StdId			= CANx_TX_HDR_STD_ID,
		.ExtId			= CANx_TX_HDR_EXT_ID,
		.IDE			= CANx_TX_HDR_IDE,
		.RTR			= CANx_TX_HDR_RTR,
		.DLC			= CANx_TX_HDR_DLC,
		.TransmitGlobalTime	= CANx_TX_HDR_TRANSMISSION_GLOBAL_TIME
	};
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
