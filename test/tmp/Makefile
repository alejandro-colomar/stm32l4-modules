#!/usr/bin/make -f

# directories

LIBALX_INC_DIR		= $(LIBALX_DIR)/inc/
STM32_CMSIS_INC_DIR	= $(STM32_CMSIS_DIR)/Include/
STM32L4_CMSIS_INC_DIR	= $(STM32L4_CMSIS_DIR)/Include/
STM32L4_HAL_INC_DIR	= $(STM32L4_HAL_DIR)/Inc/

BASE_SRC_DIR	= $(STM32L4_MODULES_DIR)/base/src/
BASE_INC_DIR	= $(STM32L4_MODULES_DIR)/inc/
DEV_SRC_DIR	= $(STM32L4_MODULES_DIR)/dev/src/
DEV_INC_DIR	= $(STM32L4_MODULES_DIR)/inc/dev/

SRC_DIR = $(STM32L4_MODULES_DIR)/test/src/
INC_DIR = $(STM32L4_MODULES_DIR)/inc/test/

# dependencies

ALL	= \
		can_test.o led_test.o tim_test.o \
		display_test.o nunchuk_test.o servo_test.o

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


CAN_INC_BASE		= can.h delay.h errors.h led.h
CAN_INC			= can_test.h
CAN_DEPS		= $(SRC_DIR)/can_test.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(BASE_INC_DIR)/%,$(CAN_INC_BASE)) \
			$(patsubst %,$(INC_DIR)/%,$(CAN_INC))
CAN_INC_DIRS		= -I $(INC_DIR) \
			-I $(BASE_INC_DIR) \
			$(COMMON_INC_DIRS)


DISPLAY_INC_BASE	= delay.h errors.h
DISPLAY_INC_DEV		= display.h
DISPLAY_INC		= display_test.h
DISPLAY_DEPS		= $(SRC_DIR)/display_test.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(BASE_INC_DIR)/%,$(DISPLAY_INC_BASE)) \
			$(patsubst %,$(DEV_INC_DIR)/%,$(DISPLAY_INC_DEV)) \
			$(patsubst %,$(INC_DIR)/%,$(DISPLAY_INC))
DISPLAY_INC_DIRS	= -I $(INC_DIR) \
			-I $(BASE_INC_DIR) \
			-I $(DEV_INC_DIR) \
			$(COMMON_INC_DIRS)


LED_INC_BASE		= delay.h errors.h led.h
LED_INC			= led_test.h
LED_DEPS		= $(SRC_DIR)/led_test.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(BASE_INC_DIR)/%,$(LED_INC_BASE)) \
			$(patsubst %,$(INC_DIR)/%,$(LED_INC))
LED_INC_DIRS		= -I $(INC_DIR) \
			-I $(BASE_INC_DIR) \
			$(COMMON_INC_DIRS)


NUNCHUK_INC_BASE	= delay.h errors.h led.h
NUNCHUK_INC_DEV		= display.h nunchuk.h servo.h
NUNCHUK_INC		= nunchuk_test.h
NUNCHUK_DEPS		= $(SRC_DIR)/nunchuk_test.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(BASE_INC_DIR)/%,$(NUNCHUK_INC_BASE)) \
			$(patsubst %,$(DEV_INC_DIR)/%,$(NUNCHUK_INC_DEV)) \
			$(patsubst %,$(INC_DIR)/%,$(NUNCHUK_INC))
NUNCHUK_INC_DIRS	= -I $(INC_DIR) \
			-I $(BASE_INC_DIR) \
			-I $(DEV_INC_DIR) \
			$(COMMON_INC_DIRS)


SERVO_INC_BASE		= delay.h errors.h
SERVO_INC_DEV		= servo.h
SERVO_INC		= servo_test.h
SERVO_DEPS		= $(SRC_DIR)/servo_test.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(BASE_INC_DIR)/%,$(SERVO_INC_BASE)) \
			$(patsubst %,$(DEV_INC_DIR)/%,$(SERVO_INC_DEV)) \
			$(patsubst %,$(INC_DIR)/%,$(SERVO_INC))
SERVO_INC_DIRS		= -I $(INC_DIR) \
			-I $(BASE_INC_DIR) \
			-I $(DEV_INC_DIR) \
			$(COMMON_INC_DIRS)

TIM_INC_LIBALX		= libalx/alx_mask.h
TIM_INC_BASE		= delay.h led.h errors.h tim.h
TIM_INC			= tim_test.h
TIM_DEPS		= $(SRC_DIR)/tim_test.c \
			$(COMMON_DEPS) \
			$(patsubst %,$(LIBALX_INC_DIR)/%,$(TIM_TST_INC_LIBALX)) \
			$(patsubst %,$(BASE_INC_DIR)/%,$(TIM_INC_BASE)) \
			$(patsubst %,$(INC_DIR)/%,$(TIM_INC))
TIM_INC_DIRS		= -I $(INC_DIR) \
			-I $(LIBALX_INC_DIR) \
			-I $(BASE_INC_DIR) \
			$(COMMON_INC_DIRS)


# target: dependencies
#	action

all: $(ALL)


can_test.s: $(CAN_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(CAN_INC_DIRS) -S $< -o $@
can_test.o: can_test.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

display_test.s: $(DISPLAY_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(DISPLAY_INC_DIRS) -S $< -o $@
display_test.o: display_test.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

led_test.s: $(LED_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(LED_INC_DIRS) -S $< -o $@
led_test.o: led_test.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

nunchuk_test.s: $(NUNCHUK_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(NUNCHUK_INC_DIRS) -S $< -o $@
nunchuk_test.o: nunchuk_test.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

servo_test.s: $(SERVO_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(SERVO_INC_DIRS) -S $< -o $@
servo_test.o: servo_test.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@

tim_test.s: $(TIM_DEPS)
	@echo	"	CC	$@"
	$(Q)$(CC) $(CFLAGS) $(TIM_INC_DIRS) -S $< -o $@
tim_test.o: tim_test.s
	@echo	"	AS	$@"
	$(Q)$(AS) $< -o $@


clean:
	$(Q)rm -f *.o *.s

################################################################################
######## End of file ###########################################################
################################################################################
