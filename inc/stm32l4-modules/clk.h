/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		clk.h
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/dec/15
 *	@brief		CLK
 */


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# ifndef		STM32L4_MODULES_CLK_H
	# define	STM32L4_MODULES_CLK_H


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/


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


/******************************************************************************
 ******* functions ************************************************************
 ******************************************************************************/
#if 0
int	sysclk_config_hse	(void);
#endif
int	sysclk_config_hsi	(void);
int	sysclk_config_msi	(void);
int	sysclk_config_pll_hsi	(void);
int	sysclk_config_pll_msi	(void);

#if 0
int	clk_hse_set		(void);
int	clk_hse_off		(void);
#endif
int	clk_hsi_set		(void);
int	clk_hsi_off		(void);
#if 0
int	clk_lse_set		(void);
int	clk_lse_off		(void);
int	clk_lsi_set		(void);
int	clk_lsi_off		(void);
#endif
int	clk_msi_set		(void);
int	clk_msi_off		(void);
int	clk_pll_set_from_hsi	(void);
int	clk_pll_set_from_msi	(void);
int	clk_pll_off		(void);

#if 0
int	clk_sysclk_set_from_hse	(void);
#endif
int	clk_sysclk_set_from_hsi	(void);
int	clk_sysclk_set_from_msi	(void);
int	clk_sysclk_set_from_pll	(void);


/******************************************************************************
 ******* include guard ********************************************************
 ******************************************************************************/
# endif			/* clk.h */


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
