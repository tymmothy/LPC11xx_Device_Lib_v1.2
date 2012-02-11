# Makefile : gmake file for building the LPC11xx Device Library v2.0
#            device library.


# Directories to make in
subdirs := src docs

.PHONY: all liblpc11xx.a docs

all: show_targets
	
O := $(if $(O),$(O),$(PWD))

show_targets:
	@echo
	@echo "Available Targets:"
	@echo
	@echo "- liblpc11xx.a -- build the library (needs MODEL, F_CPU, HSE_Val to be set)"
	@echo "- docs         -- generate docs via doxygen (doxygen must be installed)"
	@echo

liblpc11xx.a: 
	$(MAKE) -C src O=$(O) $@
	
docs:
	$(MAKE) -C docs O=$(O) $@
