# lpc11xx.mk : gmake include file for use with the LPC11xx Device Library
#
#  This file assumes use of Gmake (it uses GNU Make extensions) and gcc
#   (the GNU Compiler Collection)
#
#  This may be included in gmake files to set up compiler / linker / etc. 
#   settings for building projects using the library.  Or feel free to
#   roll your own if this doesn't meet your needs.
#
# Make environment variables that this file needs:
#
#     CROSS_COMPILE (defaults to arm-none-eabi-)
#       This is the compiler prefix.  e.g. using the default, gcc for the
#       CPU is expected to be named "arm-none-eabi-gcc" and so on...)
#
#     MODEL (no default value; must be explicitly set)
#       This is the CPU model that the project is compiling for.
#       Examples would be lpc1111, lpc1114, etc.
#
#     F_CPU (no default value; must be explicitly set)
#       This is the system frequency at which the CPU will 
#       run (NOT the external / internal oscillator speed).
#       e.g. 60000000L for 60 MHz operation.
#
#     HSE_Val (no default value; must be explicitly set)
#       This is the speed of an external crystal / oscillator
#       from which the CPU's internal (F_CPU) frequency will be derived
#       (generally 8Mhz, 12Mhz, etc.)
#
# I'm happy to take suggestions on making this all work better :/


# See if we were provided a "top level directory"; if so use that as the
#  library directory

# If not provided with top level, and not explicitly set, 
ifeq ("$(LPC11XXLIB_DIR)","")
ifeq ("$(origin T)", "command line")
  LPC11XXLIB_DIR := $(T)
else
  LPC11XXLIB_DIR := $(dir $(lastword $(MAKEFILE_LIST) ))
endif # ifeq ("$(origin T)", "command line")
endif # ifeq ("$(LPC11XXLIB_DIR)","")


# Required GMake environment variables

# Set the architecture name & compiler / linker / etc. cross compile prefix.
CROSS_COMPILE   := $(if $(CROSS_COMPILE),$(CROSS_COMPILE),arm-none-eabi-)
CPU             := $(if $(CPU),$(CPU),cortex-m0)

# If making the library, make sure that the necessary options have been set on
#  command line.

ifeq ($(filter liblpc11xx.a, $(MAKECMDGOALS)),liblpc11xx.a)
  ifeq ("$(LPC11XX_MODEL)","")
    $(error "LPC11XX_MODEL not defined.  Please set this to the model of chip (e.g. LPC11XX_MODEL=lpc1114)")
  endif

  ifeq ("$(F_CPU)","")
    $(error "F_CPU not defined.  Please set this to the target system clock speed (e.g. F_CPU=60000000L)")
  endif
endif

# For the lpc1100 device library's use
LPC11XXLIB_FLAGS := -D$(LPC11XX_MODEL) -DF_CPU=$(F_CPU) -DLPCLIB_DEBUG

ifneq ("$(HSE_Val)","")
  LPC11XXLIB_FLAGS += -DHSE_Val=$(HSE_Val)
endif

# CPU machine flags
override LPC11XX_MACHINE_FLAGS   += -mlittle-endian -mlong-calls -msoft-float \
                                    -mcpu=$(CPU) -mthumb

# Set compiler / linker flags for the library...

LPC11XX_OPTIMIZE  =  -g
LPC11XX_WARN      =  -Wall -Wextra
LPC11XX_INCLUDE   += -I$(LPC11XXLIB_DIR)/inc -I$(LPC11XXLIB_DIR)/CMSIS_2_10/CMSIS/Include
LPC11XX_CFLAGS    += $(LPC11XXLIB_FLAGS) $(LPC11XX_MACHINE_FLAGS) $(LPC11XX_INCLUDE) \
                     $(LPC11XX_OPTIMIZE) $(LPC11XX_WARN)
LPC11XX_CXXFLAGS  += $(LPC11XXLIB_FLAGS) $(LPC11XX_MACHINE_FLAGS) $(LPC11XX_INCLUDE) \
                     $(LPC11XX_OPTIMIZE) $(LPC11XX_WARN)
LPC11XX_LIBS      += -L. -llpc11xx
LPC11XX_LDFLAGS   += $(LPC11XX_MACHINE_FLAGS) -nostartfiles \
                    -T$(LPC11XXLIB_DIR)/link_scripts/$(LPC11XX_MODEL)-rom.ld
LPC11XX_ARFLAGS   := -rs

# Set defaults for general compiler / linker flags (can be overriden on command
#  line without affecting library build)

CFLAGS            =  $(LPC11XX_CFLAGS)
CXXFLAGS          =  $(LPC11XX_CXXFLAGS)
LIBS              =  $(LPC11XX_LIBS)
LDFLAGS           =  $(LPC11XX_LDFLAGS)
ASFLAGS           =  $(LPC11XX_ASFLAGS)

# Program names to use for compilation / working with binaries

AS                = $(CROSS_COMPILE)as
AR                = $(CROSS_COMPILE)ar
LD                = $(CROSS_COMPILE)ld
CC                = $(CROSS_COMPILE)gcc
CXX               = $(CROSS_COMPILE)g++
C++               = $(CXX)
STRIP             = $(CROSS_COMPILE)strip
RANLIB            = $(CROSS_COMPILE)ranlib
OBJDUMP           = $(CROSS_COMPILE)objdump
OBJCOPY           = $(CROSS_COMPILE)objcopy


# Default for building a binary.  Generally should be overriden.
%.elf: liblpc11xx.a
%.elf: %.o
	$(CC) $(LDFLAGS) $(LIBS) -o $@ $^

# Defaults for other object building...
%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -j .dataflash -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -j .dataflash -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -j .dataflash -O binary $< $@
