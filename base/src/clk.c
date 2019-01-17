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
	#include <stdbool.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/clk.h"


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


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
#if 0
	/**
	 * @brief	System Clock Configuration (from HSE)
	 * @note	Sets global variable 'prj_error'
	 */
void	sysclk_config_hse	(void)
{

	if (clk_sysclk_set_from_msi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}

	if (clk_hse_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_hse_set()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_sysclk_set_from_hse()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}
}
#endif
	/**
	 * @brief	System Clock Configuration (from MSI)
	 * @note	Sets global variable 'prj_error'
	 */
void	sysclk_config_hsi	(void)
{

	if (clk_sysclk_set_from_msi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}

	if (clk_hsi_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_hsi_set()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_sysclk_set_from_hsi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}
}

	/**
	 * @brief	System Clock Configuration (from MSI)
	 * @note	Sets global variable 'prj_error'
	 */
void	sysclk_config_msi	(void)
{

	sysclk_config_hsi();

	if (clk_msi_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_msi_set()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_sysclk_set_from_msi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}
}

	/**
	 * @brief	System Clock Configuration (from PLL from HSI)
	 * @note	Sets global variable 'prj_error'
	 */
void	sysclk_config_pll_hsi	(void)
{

	if (clk_sysclk_set_from_msi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}

	if (clk_pll_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_hsi_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_pll_set_from_hsi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_sysclk_set_from_pll()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}
}

	/**
	 * @brief	System Clock Configuration (from PLL from MSI)
	 * @note	Sets global variable 'prj_error'
	 */
void	sysclk_config_pll_msi	(void)
{

	sysclk_config_hsi();

	if (clk_pll_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_msi_off()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_pll_set_from_msi()) {
		prj_error	|= ERROR_CLK_HAL_RCC_OSC_CONF;
		while(true) {
			__NOP();
		}
	}
	if (clk_sysclk_set_from_pll()) {
		prj_error	|= ERROR_CLK_HAL_RCC_CLK_CONF;
		while(true) {
			__NOP();
		}
	}
}
#if 0
int	clk_hse_set		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_HSE,
		.HSEState		= RCC_HSE_ON,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_hse_off		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_HSE,
		.HSEState		= RCC_HSE_OFF,
		.PLL.PLLState		= RCC_PLL_NONE,
	};

	return	HAL_RCC_OscConfig(&osc);
}
#endif
int	clk_hsi_set		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_HSI,
		.HSIState		= RCC_HSI_ON,
		.HSICalibrationValue	= RCC_HSICALIBRATION_DEFAULT,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_hsi_off		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_HSI,
		.HSIState		= RCC_HSI_OFF,
		.HSICalibrationValue	= RCC_HSICALIBRATION_DEFAULT,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}
#if 0
int	clk_lse_set		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_LSE,
		.LSEState		= RCC_LSE_ON,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_lse_off		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_LSE,
		.LSEState		= RCC_LSE_OFF,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_lsi_set		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_LSI,
		.LSIState		= RCC_LSI_ON,
		.LSIDiv			= RCC_LSI_DIV1,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_lsi_off		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_LSI,
		.LSIState		= RCC_LSI_OFF,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}
#endif
int	clk_msi_set		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_MSI,
		.MSIState		= RCC_MSI_ON,
		.MSICalibrationValue	= RCC_MSICALIBRATION_DEFAULT,
		.MSIClockRange		= RCC_MSIRANGE_6,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_msi_off		(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_MSI,
		.MSIState		= RCC_MSI_OFF,
		.PLL.PLLState		= RCC_PLL_NONE
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_pll_set_from_hsi	(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_HSI,
		.HSIState		= RCC_HSI_ON,
		.HSICalibrationValue	= RCC_HSICALIBRATION_DEFAULT,
		.PLL			= {
			.PLLState		= RCC_PLL_ON,
			.PLLSource		= RCC_PLLSOURCE_HSI,
			.PLLM			= 2,
			.PLLN			= 20,
			.PLLP			= RCC_PLLP_DIV7,
			.PLLQ			= RCC_PLLQ_DIV4,
			.PLLR			= RCC_PLLR_DIV2
		}
	 };

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_pll_set_from_msi	(void)
{
	RCC_OscInitTypeDef	osc = {
		.OscillatorType		= RCC_OSCILLATORTYPE_MSI,
		.MSIState		= RCC_MSI_ON,
		.MSICalibrationValue	= RCC_MSICALIBRATION_DEFAULT,
		.MSIClockRange		= RCC_MSIRANGE_6,
		.PLL			= {
			.PLLState		= RCC_PLL_ON,
			.PLLSource		= RCC_PLLSOURCE_MSI,
			.PLLM			= 1,
			.PLLN			= 40,
			.PLLP			= RCC_PLLP_DIV7,
			.PLLQ			= RCC_PLLQ_DIV4,
			.PLLR			= RCC_PLLR_DIV2
		}
	};

	return	HAL_RCC_OscConfig(&osc);
}

int	clk_pll_off		(void)
{
	RCC_OscInitTypeDef	osc = {
		.PLL.PLLState		= RCC_PLL_OFF
	};

	return	HAL_RCC_OscConfig(&osc);
}
#if 0
int	clk_sysclk_set_from_hse	(void)
{
	RCC_ClkInitTypeDef	clk = {
		.ClockType		= RCC_CLOCKTYPE_SYSCLK |
						RCC_CLOCKTYPE_HCLK |
						RCC_CLOCKTYPE_PCLK1 |
						RCC_CLOCKTYPE_PCLK2,
		.SYSCLKSource		= RCC_SYSCLKSOURCE_HSE,
		.AHBCLKDivider		= RCC_SYSCLK_DIV1,
		.APB1CLKDivider		= RCC_HCLK_DIV1,
		.APB2CLKDivider		= RCC_HCLK_DIV1
	};

	return	HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}
#endif
int	clk_sysclk_set_from_hsi	(void)
{
	RCC_ClkInitTypeDef	clk = {
		.ClockType		= RCC_CLOCKTYPE_SYSCLK |
						RCC_CLOCKTYPE_HCLK |
						RCC_CLOCKTYPE_PCLK1 |
						RCC_CLOCKTYPE_PCLK2,
		.SYSCLKSource		= RCC_SYSCLKSOURCE_HSI,
		.AHBCLKDivider		= RCC_SYSCLK_DIV1,
		.APB1CLKDivider		= RCC_HCLK_DIV1,
		.APB2CLKDivider		= RCC_HCLK_DIV1
	};

	return	HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

int	clk_sysclk_set_from_msi	(void)
{
	RCC_ClkInitTypeDef	clk = {
		.ClockType		= RCC_CLOCKTYPE_SYSCLK |
						RCC_CLOCKTYPE_HCLK |
						RCC_CLOCKTYPE_PCLK1 |
						RCC_CLOCKTYPE_PCLK2,
		.SYSCLKSource		= RCC_SYSCLKSOURCE_MSI,
		.AHBCLKDivider		= RCC_SYSCLK_DIV1,
		.APB1CLKDivider		= RCC_HCLK_DIV1,
		.APB2CLKDivider		= RCC_HCLK_DIV1
	};

	return	HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

int	clk_sysclk_set_from_pll	(void)
{
	RCC_ClkInitTypeDef	clk = {
		.ClockType		= (RCC_CLOCKTYPE_SYSCLK |
						RCC_CLOCKTYPE_HCLK |
						RCC_CLOCKTYPE_PCLK1 |
						RCC_CLOCKTYPE_PCLK2),
		.SYSCLKSource		= RCC_SYSCLKSOURCE_PLLCLK,
		.AHBCLKDivider		= RCC_SYSCLK_DIV1,
		.APB1CLKDivider		= RCC_HCLK_DIV1,
		.APB2CLKDivider		= RCC_HCLK_DIV1
	};

	return	HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
