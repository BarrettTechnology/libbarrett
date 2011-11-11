### Makefile for the "draw2dq" WAM example program

BIN = draw2dq
OBJ = draw2dq.o control_draw2dq.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
LDFLAGS += -lbarrett -lcurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
