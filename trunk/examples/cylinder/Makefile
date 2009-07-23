### Makefile for the "cylinder" WAM example program

BIN = cylinder
OBJ = cylinder.o refgen_cylinder.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
LDFLAGS += -lbarrett -lcurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
