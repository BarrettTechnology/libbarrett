### Makefile for the "draw" WAM example program

BIN = draw
OBJ = draw.o control_draw.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
LDFLAGS += -lbarrett -lcurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
