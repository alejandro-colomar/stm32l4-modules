#!/usr/bin/make -f

# SPDX-License-Identifier:	GPL-2.0-only

################################################################################
# *AUTHORS*

# EMAIL		<1903716@gmail.com>
# FULL NAME	"Colomar Andrés, Alejandro"

# EMAIL		<@.>
# FULL NAME	"García Pedroche, Francisco Javier"

# EMAIL		<@.>
# FULL NAME	"Junquera Carrero, Santiago"

################################################################################
# Beautify output
# ---------------------------------------------------------------------------
# Prefix commands with $(Q) - that's useful
# for commands that shall be hidden in non-verbose mode.
#
#	$(Q)some command here
#
# If BUILD_VERBOSE equals 0 then the above command will be hidden.
# If BUILD_VERBOSE equals 1 then the above command is displayed.
#
# To put more focus on warnings, be less verbose as default
# Use 'make V=1' to see the full commands

ifeq ("$(origin V)","command line")
  BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
  BUILD_VERBOSE = 0
endif

ifeq ($(BUILD_VERBOSE), 1)
  Q =
else
  Q = @
endif

# If the user is running make -s (silent mode), suppress echoing of
# commands

ifneq ($(findstring s,$(filter-out --%,$(MAKEFLAGS))),)
  Q = @
endif

export	Q
export	BUILD_VERBOSE

################################################################################
# Do not print "Entering directory ...",
# but we want to display it when entering to the output directory
# so that IDEs/editors are able to understand relative filenames.
MAKEFLAGS += --no-print-directory

################################################################################
# Make variables (CC, etc...)
CC	= arm-none-eabi-gcc
AS	= arm-none-eabi-as
AR	= arm-none-eabi-ar

export	CC
export	AS
export	AR

################################################################################
# directories

STM32L4_MODULES_DIR	= $(CURDIR)
PARENT_DIR		= $(STM32L4_MODULES_DIR)/../

STM32_DRIVERS_DIR	= $(PARENT_DIR)/stm32l4-drivers/
STM32_CMSIS_DIR		= $(STM32_DRIVERS_DIR)/CMSIS/
STM32L4_CMSIS_DIR	= $(STM32_CMSIS_DIR)/ST/STM32L4xx/
STM32L4_HAL_DIR		= $(STM32_DRIVERS_DIR)/STM32L4xx_HAL_Driver/

LIBALX_DIR		= $(PARENT_DIR)/libalx/

MODULES_BASE_DIR	= $(STM32L4_MODULES_DIR)/base/
MODULES_DEV_DIR		= $(STM32L4_MODULES_DIR)/dev/
MODULES_TEST_DIR	= $(STM32L4_MODULES_DIR)/test/

INC_DIR			= $(STM32L4_MODULES_DIR)/include/
LIB_DIR			= $(STM32L4_MODULES_DIR)/lib/

export	STM32L4_MODULES_DIR

export	STM32_DRIVERS_DIR
export	STM32_CMSIS_DIR
export	STM32L4_CMSIS_DIR
export	STM32L4_HAL_DIR

export	LIBALX_DIR

export	MODULES_BASE_DIR
export	MODULES_DEV_DIR
export	MODULES_TEST_DIR

################################################################################
# target: dependencies
#	action

PHONY := all
all:
	@echo	'	MAKE	modules:	base'
	$(Q)$(MAKE) -C $(MODULES_BASE_DIR)
	@echo	'	MAKE	modules:	dev'
	$(Q)$(MAKE) -C $(MODULES_DEV_DIR)
	@echo	'	MAKE	modules:	test'
	$(Q)$(MAKE) -C $(MODULES_TEST_DIR)
	@echo	'	MAKE	modules:	lib'
	$(Q)$(MAKE) -C $(LIB_DIR)


PHONY += clean
clean:
	@echo	'	RM	*.o *.s *.a'
	$(Q)find . -type f -name '*.o' -exec rm '{}' '+'
	$(Q)find . -type f -name '*.s' -exec rm '{}' '+'
	$(Q)find . -type f -name '*.a' -exec rm '{}' '+'

################################################################################
# Declare the contents of the .PHONY variable as phony.
.PHONY: $(PHONY)




################################################################################
######## End of file ###########################################################
################################################################################
