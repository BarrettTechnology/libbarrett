### Makefile for the "ex5-simplemove" WAM example program

BIN = ex5-simplemove
OBJ = ex5-simplemove.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror
LDFLAGS += -lbarrett

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
