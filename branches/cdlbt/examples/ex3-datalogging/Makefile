### Makefile for the "ex3-datalogging" WAM example program

BIN = ex3-datalogging
OBJ = ex3-datalogging.o
CC = gcc

CFLAGS += -std=c89 -pedantic-errors -Wall -Werror
LDFLAGS += -lbarrett -lncurses

${BIN} : ${OBJ}

.PHONY : clean
clean:
	rm -f ${OBJ} ${BIN}
