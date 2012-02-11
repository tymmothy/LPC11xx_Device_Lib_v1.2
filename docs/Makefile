# gmake file for building documentation

.PHONY: all

all: docs

docs:
	doxygen Doxyfile > doxygen_out

clean:
	rm -rf html latex doxygen_out
