include ../Examples.mk
include ../../lpc11xx.mk

$(echo  !!!)

PHONY += all clean

all: liblpc11xx.a test.bin

test.elf: liblpc11xx.a

liblpc11xx.a: 
	$(MAKE) -C $(LPC11XXLIB_DIR)/src $@ O="$(PWD)"

clean:
	rm -f *.{o,a,elf,hex,srec,bin,prg,map}
