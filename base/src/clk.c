/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		clk.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/15
 *	@brief		CLK
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
/* Standard C ----------------------------------------------------------------*/
	#include <stdbool.h>

/* Drivers -------------------------------------------------------------------*/
	#include "stm32l4xx_hal.h"

/* libalx --------------------------------------------------------------------*/
/* STM32L4 modules -----------------------------------------------------------*/
	#include "errors.h"

	#include "clk.h"


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
/* Global --------------------------------------------------------------------*/
/* Static --------------------------------------------------------------------*/


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	clk_set_pll_from_msi	(void);
static	int	clk_sysclk_set_pll	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	System Clock Configuration
	 *		The system Clock is configured as follows
	 *			System Clock source            = PLL (MSI)
	 *			SYSCLK(Hz)                     = 80000000
	 *			HCLK(Hz)                       = 80000000
	 *			AHB Prescaler                  = 1
	 *			APB1 Prescaler                 = 1
	 *			APB2 Prescaler                 = 1
	 *			MSI Frequency(Hz)              = 4000000
	 *			PLL_M                          = 1
	 *			PLL_N                          = 40
	 *			PLL_R                          = 2
	 *			PLL_P                          = 7
	 *			PLL_Q                          = 4
	 *			Flash Latency(WS)              = 4
	 * @note	Sets global variable 'prj_error'
	 */
void	sysclk_config	(void)
{
	if (clk_set_pll_from_msi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}

	if (clk_sysclk_set_pll()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
	/*
	 * MSI is enabled after System reset, activate PLL with MSI as source
	 */
static	int	clk_set_pll_from_msi	(void)
{
	RCC_OscInitTypeDef	rcc_osc;

	rcc_osc.OscillatorType		= RCC_OSCILLATORTYPE_MSI;
	rcc_osc.MSIState		= RCC_MSI_ON;
	rcc_osc.MSIClockRange		= RCC_MSIRANGE_6;
	rcc_osc.MSICalibrationValue	= RCC_MSICALIBRATION_DEFAULT;
	rcc_osc.PLL.PLLState		= RCC_PLL_ON;
	rcc_osc.PLL.PLLSource		= RCC_PLLSOURCE_MSI;
	rcc_osc.PLL.PLLM		= 1u;
	rcc_osc.PLL.PLLN		= 40u;
	rcc_osc.PLL.PLLP		= RCC_PLLP_DIV7;
	rcc_osc.PLL.PLLQ		= RCC_PLLQ_DIV4;
	rcc_osc.PLL.PLLR		= RCC_PLLR_DIV2;

	return	HAL_RCC_OscConfig(&rcc_osc);
}

	/*
	 * Select PLL as system clock source and configure the HCLK, PCLK1
	 * and PCLK2 clocks dividers
	 */
static	int	clk_sysclk_set_pll	(void)
{
	RCC_ClkInitTypeDef	rcc_clk;

	rcc_clk.ClockType	= (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
				RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	rcc_clk.SYSCLKSource	= RCC_SYSCLKSOURCE_PLLCLK;
	rcc_clk.AHBCLKDivider	= RCC_SYSCLK_DIV1;
	rcc_clk.APB1CLKDivider	= RCC_HCLK_DIV1;
	rcc_clk.APB2CLKDivider	= RCC_HCLK_DIV1;

	return	HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_4);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
