### Makefile for the "draw2d" WAM example program

BIN = draw2d
OBJ = draw2d.o control_draw2d.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
LDFLAGS += -lbarrett -lcurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
