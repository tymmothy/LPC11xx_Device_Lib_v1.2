include ../../LPC11xx.mk
include ../Examples.mk

PHONY += all clean

all: libLPC11xx.a ct32b.bin

ct32b.elf: libLPC11xx.a

libLPC11xx.a: 
	$(MAKE) -C $(LPC11XXLIB_DIR)/src $@ O="$(PWD)"

clean:
	rm -f *.{o,a,elf,hex,srec,bin,prg,map}
