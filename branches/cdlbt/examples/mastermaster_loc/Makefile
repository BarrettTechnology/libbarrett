### Makefile for the "mastermaster_loc" WAM example program

BIN = mastermaster_loc
OBJ = mastermaster_loc.o refgen_mastermaster_loc.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
LDFLAGS += -lbarrett -lcurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
