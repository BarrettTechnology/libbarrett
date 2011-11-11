### Makefile for the "ex1-jointposition" WAM example program

BIN = ex1-jointposition
OBJ = ex1-jointposition.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror
LDFLAGS += -lbarrett

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
