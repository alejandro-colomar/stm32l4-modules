#!/usr/bin/make -f

# MACRO = substitute with this

# directories

BASE_TMP_DIR	= $(MODULES_BASE_DIR)/tmp/
DEV_TMP_DIR	= $(MODULES_DEV_DIR)/tmp/
TEST_TMP_DIR	= $(MODULES_TEST_DIR)/tmp/

# dependencies

ALL		= \
		libstm32l4-modules-base.a \
		libstm32l4-modules-dev.a \
		libstm32l4-modules-test.a

MODS_OBJ_BASE	= \
		button.o can.o clk.o delay.o errors.o i2c.o led.o pwm.o spi.o \
		tim.o
MODS_BASE_OBJS	= $(patsubst %,$(BASE_TMP_DIR)/%,$(MODS_OBJ_BASE))


MODS_OBJ_DEV	= display.o nunchuk.o servo.o
MODS_DEV_OBJS	= $(patsubst %,$(DEV_TMP_DIR)/%,$(MODS_OBJ_DEV))


MODS_OBJ_TEST	= \
		can_test.o led_test.o tim_test.o \
		display_test.o nunchuk_test.o servo_test.o
MODS_TEST_OBJS	= $(patsubst %,$(TEST_TMP_DIR)/%,$(MODS_OBJ_TEST))

# target: dependencies
#	action

all: $(ALL)


libstm32l4-modules-base.a: $(MODS_BASE_OBJS)
	@echo	"	AR	$@"
	$(Q)$(AR) -r -c -s -o $@ $^

libstm32l4-modules-dev.a: $(MODS_DEV_OBJS)
	@echo	"	AR	$@"
	$(Q)$(AR) -r -c -s -o $@ $^

libstm32l4-modules-test.a: $(MODS_TEST_OBJS)
	@echo	"	AR	$@"
	$(Q)$(AR) -r -c -s -o $@ $^
