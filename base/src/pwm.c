/******************************************************************************
 *	Copyright (C) 2018	Colomar Andrés, Alejandro		      *
 *	Copyright (C) 2018	García Pedroche, Francisco Javier	      *
 *	Copyright (C) 2018	Junquera Carrero, Santiago		      *
 *	SPDX-License-Identifier:	LGPL-2.0-only			      *
 ******************************************************************************/

/**
 *	@file		pwm.c
 *	@author		Colomar Andrés, Alejandro
 *	@author		García Pedroche, Francisco Javier
 *	@author		Junquera Carrero, Santiago
 *	@copyright	LGPL-2.0-only
 *	@date		2018/nov/27
 *	@brief		PWM
 */


/******************************************************************************
 ******* headers **************************************************************
 ******************************************************************************/
	#include <stdbool.h>
	#include <stdint.h>

	#include "stm32l4xx_hal.h"

	#include "stm32l4-modules/errors.h"

	#include "stm32l4-modules/pwm.h"


/******************************************************************************
 ******* macros ***************************************************************
 ******************************************************************************/
# define	TIMx_INSTANCE			(TIM2)
# define	TIMx_CLK_ENABLE()		__HAL_RCC_TIM2_CLK_ENABLE()
# define	TIMx_CLK_DISABLE()		__HAL_RCC_TIM2_CLK_DISABLE()

# define	TIMx_GPIO_MODE			(GPIO_MODE_AF_OD)
/*# define	TIMx_GPIO_MODE			(GPIO_MODE_AF_PP)*/

# define	TIMx_CH1_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
# define	TIMx_CH1_GPIO_PORT		(GPIOA)
# define	TIMx_CH1_GPIO_PIN		(GPIO_PIN_15)
# define	TIMx_CH1_GPIO_MODE		(TIMx_GPIO_MODE)
# define	TIMx_CH1_GPIO_PULL		(GPIO_NOPULL)
# define	TIMx_CH1_GPIO_SPEED		(GPIO_SPEED_FREQ_LOW)
# define	TIMx_CH1_GPIO_ALT		(GPIO_AF1_TIM2)

# define	TIMx_CH2_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
# define	TIMx_CH2_GPIO_PORT		(GPIOA)
# define	TIMx_CH2_GPIO_PIN		(GPIO_PIN_1)
# define	TIMx_CH2_GPIO_MODE		(TIMx_GPIO_MODE)
# define	TIMx_CH2_GPIO_PULL		(GPIO_NOPULL)
# define	TIMx_CH2_GPIO_SPEED		(GPIO_SPEED_FREQ_LOW)
# define	TIMx_CH2_GPIO_ALT		(GPIO_AF1_TIM2)

# define	TIMx_CH3_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
# define	TIMx_CH3_GPIO_PORT		(GPIOB)
# define	TIMx_CH3_GPIO_PIN		(GPIO_PIN_10)
# define	TIMx_CH3_GPIO_MODE		(TIMx_GPIO_MODE)
# define	TIMx_CH3_GPIO_PULL		(GPIO_NOPULL)
# define	TIMx_CH3_GPIO_SPEED		(GPIO_SPEED_FREQ_LOW)
# define	TIMx_CH3_GPIO_ALT		(GPIO_AF1_TIM2)

# define	TIMx_CH4_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
# define	TIMx_CH4_GPIO_PORT		(GPIOB)
# define	TIMx_CH4_GPIO_PIN		(GPIO_PIN_11)
# define	TIMx_CH4_GPIO_MODE		(TIMx_GPIO_MODE)
# define	TIMx_CH4_GPIO_PULL		(GPIO_NOPULL)
# define	TIMx_CH4_GPIO_SPEED		(GPIO_SPEED_FREQ_LOW)
# define	TIMx_CH4_GPIO_ALT		(GPIO_AF1_TIM2)


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
static	TIM_HandleTypeDef	tim;
static	TIM_OC_InitTypeDef	oc_init;


/******************************************************************************
 ******* static functions (prototypes) ****************************************
 ******************************************************************************/
static	int	pwm_timx_clk_conf		(void);
static	int	pwm_timx_tim_init		(uint32_t resolution_sec,
								uint32_t period);
static	int	pwm_timx_tim_deinit		(void);
static	int	pwm_timx_master_conf		(void);
static	void	pwm_timx_oc_conf		(void);
static	void	pwm_timx_ch1_gpio_init		(void);
static	void	pwm_timx_ch1_gpio_deinit	(void);
static	void	pwm_timx_ch2_gpio_init		(void);
static	void	pwm_timx_ch2_gpio_deinit	(void);
static	void	pwm_timx_ch3_gpio_init		(void);
static	void	pwm_timx_ch3_gpio_deinit	(void);
static	void	pwm_timx_ch4_gpio_init		(void);
static	void	pwm_timx_ch4_gpio_deinit	(void);


/******************************************************************************
 ******* global functions *****************************************************
 ******************************************************************************/
	/**
	 * @brief	Initialize PWM using TIMx
	 * @param	resolution_sec:	divisions in 1 s
	 * @param	period:		period of the pwm (in resolution_sec units)
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	pwm_timx_init		(uint32_t resolution_sec, uint32_t period)
{

	if (init_pending) {
		init_pending	= false;
	} else {
		return	ERROR_OK;
	}

	TIMx_CLK_ENABLE();
	if (pwm_timx_clk_conf()) {
		prj_error	|= ERROR_PWM_HAL_TIM_CLK_CONF;
		prj_error_handle();
		goto err_clk_conf;
	}
	if (pwm_timx_tim_init(resolution_sec, period)) {
		prj_error	|= ERROR_PWM_HAL_TIM_PWM_INIT;
		prj_error_handle();
		goto err_init;
	}
	if (pwm_timx_master_conf()) {
		prj_error	|= ERROR_PWM_HAL_TIM_MASTER_CONF;
		prj_error_handle();
		goto err_master_conf;
	}
	pwm_timx_oc_conf();

	pwm_timx_ch1_gpio_init();
	pwm_timx_ch2_gpio_init();
	pwm_timx_ch3_gpio_init();
	pwm_timx_ch4_gpio_init();

	return	ERROR_OK;


err_master_conf:
	if (pwm_timx_tim_deinit()) {
		prj_error	|= ERROR_PWM_HAL_TIM_PWM_DEINIT;
		prj_error_handle();
	}

err_init:
err_clk_conf:
	TIMx_CLK_DISABLE();
	init_pending	= true;

	return	ERROR_NOK;
}

	/**
	 * @brief	Deinitialize PWM using TIMx
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	pwm_timx_deinit		(void)
{
	int	status;

	status	= ERROR_OK;

	if (!init_pending) {
		init_pending	= true;
	} else {
		return	status;
	}

	pwm_timx_ch4_gpio_deinit();
	pwm_timx_ch3_gpio_deinit();
	pwm_timx_ch2_gpio_deinit();
	pwm_timx_ch1_gpio_deinit();

	if (pwm_timx_tim_deinit()) {
		prj_error	|= ERROR_PWM_HAL_TIM_PWM_DEINIT;
		prj_error_handle();
		status	= ERROR_NOK;
	}
	TIMx_CLK_DISABLE();

	return	status;
}

	/**
	 * @brief	Set PWM using TIMx
	 * @param	duty_cycle:	duty cycle value (fraction)
	 * @param	tim_chan:	channel to be used (1 through 4)
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	pwm_timx_chX_set	(uint32_t tim_chan, float duty_cycle)
{

	if (init_pending) {
		prj_error	|= ERROR_PWM_INIT;
		prj_error_handle();
		return	ERROR_NOK;
	}

	if (duty_cycle > 1.0) {
		oc_init.Pulse	= tim.Init.Period;
		prj_error	|= ERROR_PWM_DUTY;
	} else if (duty_cycle < 0.0) {
		oc_init.Pulse	= 0;
		prj_error	|= ERROR_PWM_DUTY;
	} else {
		oc_init.Pulse	= tim.Init.Period * duty_cycle;
	}

	if (HAL_TIM_PWM_ConfigChannel(&tim, &oc_init, tim_chan)) {
		prj_error	|= ERROR_PWM_HAL_TIM_PWM_CONF;
		prj_error_handle();
		return	ERROR_NOK;
	}

	if (HAL_TIM_PWM_Start(&tim, tim_chan)) {
		prj_error	|= ERROR_PWM_HAL_TIM_PWM_START;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}

	/**
	 * @brief	Stop PWM using TIM2
	 * @param	tim_chan:	channel to be used (1 through 4)
	 * @return	Error
	 * @note	Sets global variable 'prj_error'
	 */
int	pwm_timx_chX_stop	(uint32_t tim_chan)
{

	if (init_pending) {
		prj_error	|= ERROR_PWM_INIT;
		prj_error_handle();
		return	ERROR_NOK;
	}

	if (HAL_TIM_PWM_Stop(&tim, tim_chan)) {
		prj_error	|= ERROR_PWM_HAL_TIM_PWM_STOP;
		prj_error_handle();
		return	ERROR_NOK;
	}

	return	ERROR_OK;
}


/******************************************************************************
 ******* static functions (definitions) ***************************************
 ******************************************************************************/
static	int	pwm_timx_clk_conf		(void)
{
	TIM_ClockConfigTypeDef	clk;

	clk.ClockSource		= TIM_CLOCKSOURCE_INTERNAL;
	clk.ClockPolarity	= TIM_CLOCKPOLARITY_INVERTED;
	clk.ClockPrescaler	= TIM_CLOCKPRESCALER_DIV1;
	clk.ClockFilter		= 0;

	return	HAL_TIM_ConfigClockSource(&tim, &clk);
}

static	int	pwm_timx_tim_init		(uint32_t resolution_sec,
								uint32_t period)
{

	tim.Instance		= TIMx_INSTANCE;
	tim.Init.Prescaler		= (SystemCoreClock / resolution_sec)
									- 1u;
	tim.Init.CounterMode		= TIM_COUNTERMODE_UP;
	tim.Init.Period			= period - 1u;
	tim.Init.ClockDivision		= TIM_CLOCKDIVISION_DIV1;
	tim.Init.RepetitionCounter	= 0;
	tim.Init.AutoReloadPreload	= TIM_AUTORELOAD_PRELOAD_DISABLE;

	return	HAL_TIM_PWM_Init(&tim);
}

static	int	pwm_timx_tim_deinit		(void)
{

	return	HAL_TIM_PWM_DeInit(&tim);
}

	/* FIXME:  Is it needed? */
static	int	pwm_timx_master_conf		(void)
{
	TIM_MasterConfigTypeDef	master;

	master.MasterOutputTrigger	= TIM_TRGO_RESET;
	master.MasterOutputTrigger2	= TIM_TRGO2_RESET;
	master.MasterSlaveMode		= TIM_MASTERSLAVEMODE_DISABLE;

	return	HAL_TIMEx_MasterConfigSynchronization(&tim, &master);
}

static	void	pwm_timx_oc_conf		(void)
{

	oc_init.OCMode		= TIM_OCMODE_PWM1;
	oc_init.OCPolarity	= TIM_OCPOLARITY_HIGH;
	oc_init.OCFastMode	= TIM_OCFAST_DISABLE;
	oc_init.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	oc_init.OCNIdleState	= TIM_OCNIDLESTATE_RESET;
	oc_init.OCIdleState	= TIM_OCIDLESTATE_RESET;
	oc_init.OCIdleState	= TIM_OCIDLESTATE_RESET;
}

static	void	pwm_timx_ch1_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	TIMx_CH1_GPIO_CLK_ENABLE();
	gpio.Pin	= TIMx_CH1_GPIO_PIN;
	gpio.Mode	= TIMx_CH1_GPIO_MODE;
	gpio.Pull	= TIMx_CH1_GPIO_PULL;
	gpio.Speed	= TIMx_CH1_GPIO_SPEED;
	gpio.Alternate	= TIMx_CH1_GPIO_ALT;
	HAL_GPIO_Init(TIMx_CH1_GPIO_PORT, &gpio);
}

static	void	pwm_timx_ch1_gpio_deinit	(void)
{

	HAL_GPIO_DeInit(TIMx_CH1_GPIO_PORT, TIMx_CH1_GPIO_PIN);
}

static	void	pwm_timx_ch2_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	TIMx_CH2_GPIO_CLK_ENABLE();
	gpio.Pin	= TIMx_CH2_GPIO_PIN;
	gpio.Mode	= TIMx_CH2_GPIO_MODE;
	gpio.Pull	= TIMx_CH2_GPIO_PULL;
	gpio.Speed	= TIMx_CH2_GPIO_SPEED;
	gpio.Alternate	= TIMx_CH2_GPIO_ALT;
	HAL_GPIO_Init(TIMx_CH2_GPIO_PORT, &gpio);
}

static	void	pwm_timx_ch2_gpio_deinit	(void)
{

	HAL_GPIO_DeInit(TIMx_CH2_GPIO_PORT, TIMx_CH2_GPIO_PIN);
}

static	void	pwm_timx_ch3_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	TIMx_CH3_GPIO_CLK_ENABLE();
	gpio.Pin	= TIMx_CH3_GPIO_PIN;
	gpio.Mode	= TIMx_CH3_GPIO_MODE;
	gpio.Pull	= TIMx_CH3_GPIO_PULL;
	gpio.Speed	= TIMx_CH3_GPIO_SPEED;
	gpio.Alternate	= TIMx_CH3_GPIO_ALT;
	HAL_GPIO_Init(TIMx_CH3_GPIO_PORT, &gpio);
}

static	void	pwm_timx_ch3_gpio_deinit	(void)
{

	HAL_GPIO_DeInit(TIMx_CH3_GPIO_PORT, TIMx_CH3_GPIO_PIN);
}

static	void	pwm_timx_ch4_gpio_init		(void)
{
	GPIO_InitTypeDef	gpio;

	TIMx_CH4_GPIO_CLK_ENABLE();
	gpio.Pin	= TIMx_CH4_GPIO_PIN;
	gpio.Mode	= TIMx_CH4_GPIO_MODE;
	gpio.Pull	= TIMx_CH4_GPIO_PULL;
	gpio.Speed	= TIMx_CH4_GPIO_SPEED;
	gpio.Alternate	= TIMx_CH4_GPIO_ALT;
	HAL_GPIO_Init(TIMx_CH4_GPIO_PORT, &gpio);
}

static	void	pwm_timx_ch4_gpio_deinit	(void)
{

	HAL_GPIO_DeInit(TIMx_CH4_GPIO_PORT, TIMx_CH4_GPIO_PIN);
}


/******************************************************************************
 ******* end of file **********************************************************
 ******************************************************************************/
