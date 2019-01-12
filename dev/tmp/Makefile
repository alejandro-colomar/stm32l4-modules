#!/usr/bin/make -f

# directories

LIBALX_INC_DIR		= $(LIBALX_DIR)/inc/
STM32_CMSIS_INC_DIR	= $(STM32_CMSIS_DIR)/Include/
STM32L4_CMSIS_INC_DIR	= $(STM32L4_CMSIS_DIR)/Include/
STM32L4_HAL_INC_DIR	= $(STM32L4_HAL_DIR)/Inc/

SRC_DIR = $(STM32L4_MODULES_DIR)/dev/src/
INC_DIR = $(STM32L4_MODULES_DIR)/inc/

# dependencies

ALL	= display.o nunchuk.o servo.o

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


DISPLAY_INC		= stm32l4-modules/dev/display.h \
				stm32l4-modules/errors.h \
				stm32l4-modules/spi.h
DISPLAY_DEPS		= $(SRC_DIR)/display.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(DISPLAY_INC))
DISPLAY_INC_DIRS	= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


NUNCHUK_INC		= stm32l4-modules/dev/nunchuk.h \
				stm32l4-modules/errors.h \
				stm32l4-modules/i2c.h
NUNCHUK_DEPS		= $(SRC_DIR)/nunchuk.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(INC_DIR)/%,$(NUNCHUK_INC))
NUNCHUK_INC_DIRS	= -I $(INC_DIR) \
			$(COMMON_INC_DIRS)


SERVO_INC_LIBALX	= libalx/alx_math.h
SERVO_INC_BASE		=
SERVO_INC		= stm32l4-modules/dev/servo.h \
				stm32l4-modules/errors.h \
				stm32l4-modules/pwm.h
SERVO_DEPS		= $(SRC_DIR)/servo.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(LIBALX_INC_DIR)/%,$(SERVO_INC_LIBALX)) \
			$(patsubst %,$(INC_DIR)/%,$(SERVO_INC))
SERVO_INC_DIRS		= -I $(INC_DIR) \
			-I $(LIBALX_INC_DIR) \
			$(COMMON_INC_DIRS)

# target: dependencies
#	action

all: $(ALL)


display.s: $(DISPLAY_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(DISPLAY_INC_DIRS) -S $< -o $@
display.o: display.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

nunchuk.s: $(NUNCHUK_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(NUNCHUK_INC_DIRS) -S $< -o $@
nunchuk.o: nunchuk.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

servo.s: $(SERVO_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(SERVO_INC_DIRS) -S $< -o $@
servo.o: servo.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@


clean:
	$(Q)rm -f *.o *.s

################################################################################
######## End of file ###########################################################
################################################################################
