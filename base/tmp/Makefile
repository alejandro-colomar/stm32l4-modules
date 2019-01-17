#!/usr/bin/make -f

# directories

LIBALX_INC_DIR		= $(LIBALX_DIR)/inc/
STM32_CMSIS_INC_DIR	= $(STM32_CMSIS_DIR)/Include/
STM32L4_CMSIS_INC_DIR	= $(STM32L4_CMSIS_DIR)/Include/
STM32L4_HAL_INC_DIR	= $(STM32L4_HAL_DIR)/Inc/

INC_DIR = $(STM32L4_MODULES_DIR)/inc/
SRC_DIR = $(STM32L4_MODULES_DIR)/base/src/

# dependencies

ALL	= button.o can.o clk.o delay.o delay_it.o errors.o i2c.o led.o pwm.o spi.o tim.o

COMMON_INC_CMSIS	= core_cm4.h cmsis_gcc.h
COMMON_INC_CMSIS_L4	= stm32l4xx.h
COMMON_INC_HAL		= stm32l4xx_hal.h
COMMON_DEPS		= \
			$(patsubst %,$(STM32_CMSIS_INC_DIR)/%,$(COMMON_INC_CMSIS)) \
			$(patsubst %,$(STM32L4_CMSIS_INC_DIR)/%,$(COMMON_INC_CMSIS_L4)) \
			$(patsubst %,$(STM32L4_HAL_INC_DIR)/%,$(COMMON_INC_HAL))
COMMON_INC_DIRS		= \
			-I $(STM32_CMSIS_INC_DIR) \
			-I $(STM32L4_CMSIS_INC_DIR) \
			-I $(STM32L4_HAL_INC_DIR)


BTN_INC			= stm32l4-modules/button.h \
				stm32l4-modules/errors.h
BTN_DEPS		= $(SRC_DIR)/button.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(BTN_INC))
BTN_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


CAN_INC			= stm32l4-modules/can.h	\
				stm32l4-modules/errors.h
CAN_DEPS		= $(SRC_DIR)/can.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(CAN_INC))
CAN_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


CLK_INC			= stm32l4-modules/clk.h	\
				stm32l4-modules/errors.h
CLK_DEPS		= $(SRC_DIR)/clk.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(CLK_INC))
CLK_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


DELAY_INC		= stm32l4-modules/delay.h \
				stm32l4-modules/errors.h
DELAY_DEPS		= $(SRC_DIR)/delay.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(DELAY_INC))
DELAY_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


DEL_IT_INC		= stm32l4-modules/delay_it.h \
				stm32l4-modules/tim.h \
				stm32l4-modules/errors.h
DEL_IT_DEPS		= $(SRC_DIR)/delay_it.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(DEL_IT_INC))
DEL_IT_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


ERRORS_INC_LIBALX	= libalx/alx_mask.h
ERRORS_INC		= stm32l4-modules/errors.h \
				stm32l4-modules/delay.h \
				stm32l4-modules/led.h
ERRORS_DEPS		= $(SRC_DIR)/errors.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(LIBALX_INC_DIR)/%,$(ERRORS_INC_LIBALX)) \
			$(patsubst %,$(INC_DIR)/%,$(ERRORS_INC))
ERRORS_INC_DIRS		= -I $(INC_DIR) \
			-I $(LIBALX_INC_DIR) \
			$(COMMON_INC_DIRS)


I2C_INC			= stm32l4-modules/i2c.h	\
				stm32l4-modules/errors.h
I2C_DEPS		= $(SRC_DIR)/i2c.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(I2C_INC))
I2C_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


LED_INC			= stm32l4-modules/led.h \
				stm32l4-modules/errors.h
LED_DEPS		= $(SRC_DIR)/led.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(LED_INC))
LED_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


PWM_INC			= stm32l4-modules/pwm.h	\
				stm32l4-modules/errors.h
PWM_DEPS		= $(SRC_DIR)/pwm.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(PWM_INC))
PWM_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


SPI_INC			= stm32l4-modules/spi.h	\
				stm32l4-modules/errors.h
SPI_DEPS		= $(SRC_DIR)/spi.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(SPI_INC))
SPI_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)

TIM_INC			= stm32l4-modules/tim.h	\
				stm32l4-modules/errors.h
TIM_DEPS		= $(SRC_DIR)/tim.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(TIM_INC))
TIM_INC_DIRS		= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)

# target: dependencies
#	action

all: $(ALL)


button.s: $(BTN_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(BTN_INC_DIRS) -S $< -o $@
button.o: button.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

can.s: $(CAN_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(CAN_INC_DIRS) -S $< -o $@
can.o: can.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

clk.s: $(CLK_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(CLK_INC_DIRS) -S $< -o $@
clk.o: clk.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

delay.s: $(DELAY_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(DELAY_INC_DIRS) -S $< -o $@
delay.o: delay.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

delay_it.s: $(DEL_IT_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(DEL_IT_INC_DIRS) -S $< -o $@
delay_it.o: delay_it.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

errors.s: $(ERRORS_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(ERRORS_INC_DIRS) -S $< -o $@
errors.o: errors.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

i2c.s: $(I2C_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(I2C_INC_DIRS) -S $< -o $@
i2c.o: i2c.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

led.s: $(LED_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(LED_INC_DIRS) -S $< -o $@
led.o: led.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

pwm.s: $(PWM_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(PWM_INC_DIRS) -S $< -o $@
pwm.o: pwm.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

spi.s: $(SPI_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(SPI_INC_DIRS) -S $< -o $@
spi.o: spi.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

tim.s: $(TIM_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(TIM_INC_DIRS) -S $< -o $@
tim.o: tim.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@


clean:
	$(Q)rm -f *.o *.s

################################################################################
######## End of file ###########################################################
################################################################################
