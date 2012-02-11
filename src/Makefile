# Makefile : gmake file for building the LPC11xx Device Library v2.0
#            device library proper.
#
# Author: Tymm Twillman <tymm@gmail.com>
# Date:   29. December 2011

# Prevent warnings about overriding commands...
LPC11XX_RECURSE=1


# Check to see if we were passed a "top directory"
ifneq ("$(origin T)", "command line")
  T := $(dir $(CURDIR))
endif

# Include required compiler / environment settings
include $(T)/lpc11xx.mk

# Set the search path...
vpath %.c $(T)/src
vpath %.s $(T)/src
vpath %.h $(T)/inc

# If called from out of the tree, run make from the directory
#  the build was called from, so output files end up there.
ifeq ("$(origin O)", "command line")
  BUILD_OUTPUT := $(O)
endif

ifneq ("$(origin T)", "command line")
ifneq ($(BUILD_OUTPUT),)

PHONY += $(MAKECMDGOALS) sub-make

$(filter-out _all sub-make $(CURDIR)/Makefile, $(MAKECMDGOALS)) _all: sub-make
	@:

sub-make:
	$(MAKE) -C $(BUILD_OUTPUT) T=$(dir $(CURDIR)) -f $(CURDIR)/Makefile \
	           $(filter-out _all sub-make,$(MAKECMDGOALS))

skip-makefile := 1
endif
endif

ifeq ($(skip-makefile),)

# Standard archive creation flags for GNU ar
ARFLAGS := $(if $(ARFLAGS),$(ARFLAGS),-urls)


# Dependencies / object files for the library
liblpc11xx_SRC := lpc11xx_crp.c lpc11xx_iap.c lpc11xx_pll.c system_lpc11xx.c \
                  lpclib_assert.c 
liblpc11xx_OBJ := $(liblpc11xx_SRC:.c=.o) lpc11xx_crt0.o


.PHONY: all

all: liblpc11xx.a

liblpc11xx.a: $(liblpc11xx_OBJ)
	$(AR) $(ARFLAGS) $@ $^

.PHONY: clean

clean:
	rm -f $(liblpc11xx_OBJ) liblpc11xx.a


endif # ifeq ($(skip-makefile),)

